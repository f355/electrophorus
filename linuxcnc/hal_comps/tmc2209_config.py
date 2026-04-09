#!/usr/bin/env python3
"""LinuxCNC userspace HAL component for TMC2209 stepper driver configuration.

Configures TMC2209 drivers via UART and monitors driver status
(temperature warnings, short/open-load faults) during operation.

This component does NOT handle step/dir generation — that is done by the
realtime electrophorus component. This is purely for register configuration
and diagnostics.

The component waits for the 'enable' input pin to go high before attempting
any UART communication. This is necessary because the TMC2209 drivers are
typically unpowered until e-stop is released. When power is cycled (e-stop
press/release), the drivers lose all register state, so we re-configure
every time 'enable' transitions from low to high.

Usage in HAL:
    loadusr -W tmc2209_config -n tmc2209 \\
        --serial /dev/ttyAMA0 --baud 115200 \\
        --addr 0,1 --names x,z

    # per-motor configuration via HAL params (set before enable)
    setp tmc2209.x.current-rms   1900
    setp tmc2209.x.microsteps    8
    setp tmc2209.x.spreadcycle   1
    setp tmc2209.z.current-rms   1900
    setp tmc2209.z.microsteps    8
    setp tmc2209.z.spreadcycle   1

    # gate on e-stop / power enable so we only configure when drivers have power
    net estop.user-enable-out => tmc2209.enable
"""

import argparse
import hal
import logging
import sys
import time

from tmc_driver import Tmc2209, Loglevel
from tmc_driver.com import TmcComUart

POLL_INTERVAL_S = 1.0
COMPONENT_NAME = "tmc2209"

log = logging.getLogger("tmc2209_config")


def parse_args():
    p = argparse.ArgumentParser(description="TMC2209 HAL configurator")
    p.add_argument("-n", "--name", default=COMPONENT_NAME,
                   help="HAL component name (default: tmc2209)")
    p.add_argument("--serial", default="/dev/ttyAMA0",
                   help="Serial port for TMC2209 UART (default: /dev/ttyAMA0)")
    p.add_argument("--baud", type=int, default=115200,
                   help="UART baud rate (default: 115200)")
    p.add_argument("--addr", default="0,1",
                   help="Comma-separated driver addresses (default: 0,1)")
    p.add_argument("--names", default=None,
                   help="Comma-separated axis names (default: same as addr)")
    return p.parse_args()


# Per-motor HAL params: (suffix, hal_type, default, description)
MOTOR_PARAMS = [
    ("current-rms",     hal.HAL_S32,   1900,  "Run current in mA RMS"),
    ("hold-multiplier", hal.HAL_FLOAT, 0.5,   "Hold current as fraction of run current"),
    ("hold-delay",      hal.HAL_S32,   6,     "Delay before switching to hold current (0-15)"),
    ("microsteps",      hal.HAL_S32,   8,     "Microstep resolution: 1,2,4,8,16,32,64,128,256"),
    ("interpolation",   hal.HAL_BIT,   True,  "Internal 256-microstep interpolation"),
    ("spreadcycle",     hal.HAL_BIT,   True,  "1=SpreadCycle (robust), 0=StealthChop (quiet)"),
    # Chopper tuning
    ("toff",     hal.HAL_S32, 3,
     "Slow-decay duration per PWM cycle. Higher=quieter but more ripple. "
     "0=driver disabled. Range 1-15"),
    ("hstrt",    hal.HAL_S32, 5,
     "Added to hend to set fast-decay phase width. "
     "Higher=more aggressive current control, noisier. Range 0-7"),
    ("hend",     hal.HAL_S32, 0,
     "Comparator threshold ending the fast-decay phase. "
     "Higher=more energy per cycle, better torque at speed. Range -3..12"),
    ("tbl",      hal.HAL_S32, 2,
     "Comparator blank time — ignores measurement noise after switching. "
     "Longer=cleaner signal, slower response. 0-3: 16/24/36/54 clocks"),
    ("tpwmthrs", hal.HAL_S32, 0,
     "Velocity threshold for automatic StealthChop/SpreadCycle switching. "
     "Below=quiet StealthChop, above=strong SpreadCycle. 0=disabled"),
]


def configure_driver(tmc, h, axis_name):
    """Apply configuration to a single TMC2209 driver, reading from HAL params."""
    p = axis_name  # HAL param prefix
    log.info("%s: configuring TMC2209 (addr %d)", axis_name, tmc.tmc_com.driver_address)

    # Current
    current_rms = int(h[f"{p}.current-rms"])
    actual = tmc.set_current_rms(
        current_rms,
        hold_current_multiplier=float(h[f"{p}.hold-multiplier"]),
        hold_current_delay=int(h[f"{p}.hold-delay"]),
    )
    log.info("%s: current set to %d mA RMS (requested %d)", axis_name, actual, current_rms)

    # Microsteps + interpolation
    microsteps = int(h[f"{p}.microsteps"])
    tmc.set_microstepping_resolution(microsteps)
    use_interpolation = bool(h[f"{p}.interpolation"])
    tmc.set_interpolation(use_interpolation)
    log.info("%s: %d microsteps, interpolation=%s", axis_name, microsteps, use_interpolation)

    # Chopper mode
    use_spreadcycle = bool(h[f"{p}.spreadcycle"])
    tmc.set_spreadcycle(use_spreadcycle)
    log.info("%s: chopper mode = %s", axis_name, "SpreadCycle" if use_spreadcycle else "StealthChop")

    # Chopper tuning
    toff = int(h[f"{p}.toff"])
    hstrt = int(h[f"{p}.hstrt"])
    hend = int(h[f"{p}.hend"])
    tbl = int(h[f"{p}.tbl"])
    tmc.chopconf.read()
    tmc.chopconf.toff = toff
    tmc.chopconf.hstrt = hstrt
    tmc.chopconf.hend = hend
    tmc.chopconf.tbl = tbl
    tmc.chopconf.write_check()
    log.info("%s: CHOPCONF toff=%d hstrt=%d hend=%d tbl=%d", axis_name, toff, hstrt, hend, tbl)

    # Hybrid threshold
    tpwmthrs = int(h[f"{p}.tpwmthrs"])
    if tpwmthrs > 0:
        tmc.set_tpwmthrs(tpwmthrs)
        log.info("%s: TPWMTHRS=%d", axis_name, tpwmthrs)

    # Verify IFCNT incremented (driver is alive)
    ifcnt = tmc.get_interface_transmission_counter()
    log.info("%s: IFCNT=%d (driver is responding)", axis_name, ifcnt)


def create_hal_pins(h, names):
    """Create all HAL pins/params: global enable/configured + per-driver config and status."""
    # Global input: gate all UART communication on this pin
    h.newpin("enable", hal.HAL_BIT, hal.HAL_IN)
    # Global output: True once all drivers have been successfully configured
    h.newpin("configured", hal.HAL_BIT, hal.HAL_OUT)

    for axis_name in names:
        p = axis_name

        # Per-motor configuration params (set via setp before enable)
        for suffix, hal_type, default, _desc in MOTOR_PARAMS:
            h.newparam(f"{p}.{suffix}", hal_type, hal.HAL_RW)
            h[f"{p}.{suffix}"] = default

        # Status output pins (updated from DRV_STATUS polling)
        h.newpin(f"{p}.overtemp-warning", hal.HAL_BIT, hal.HAL_OUT)
        h.newpin(f"{p}.overtemp-shutdown", hal.HAL_BIT, hal.HAL_OUT)
        h.newpin(f"{p}.short-to-ground-a", hal.HAL_BIT, hal.HAL_OUT)
        h.newpin(f"{p}.short-to-ground-b", hal.HAL_BIT, hal.HAL_OUT)
        h.newpin(f"{p}.open-load-a", hal.HAL_BIT, hal.HAL_OUT)
        h.newpin(f"{p}.open-load-b", hal.HAL_BIT, hal.HAL_OUT)
        h.newpin(f"{p}.standstill", hal.HAL_BIT, hal.HAL_OUT)
        h.newpin(f"{p}.cs-actual", hal.HAL_S32, hal.HAL_OUT)
        h.newpin(f"{p}.fault", hal.HAL_BIT, hal.HAL_OUT)



def poll_status(tmc, h, axis_name):
    """Read DRV_STATUS and update HAL pins."""
    prefix = axis_name
    try:
        tmc.drvstatus.read()
        ds = tmc.drvstatus

        h[f"{prefix}.overtemp-warning"] = ds.otpw
        h[f"{prefix}.overtemp-shutdown"] = ds.ot
        h[f"{prefix}.short-to-ground-a"] = ds.s2ga
        h[f"{prefix}.short-to-ground-b"] = ds.s2gb
        h[f"{prefix}.open-load-a"] = ds.ola
        h[f"{prefix}.open-load-b"] = ds.olb
        h[f"{prefix}.standstill"] = ds.stst
        h[f"{prefix}.cs-actual"] = ds.cs_actual
        # Fault = any critical condition
        h[f"{prefix}.fault"] = ds.ot or ds.s2ga or ds.s2gb or ds.s2vsa or ds.s2vsb
    except Exception as e:
        log.warning("%s: DRV_STATUS read failed: %s", axis_name, e)
        h[f"{prefix}.fault"] = True


def init_drivers(args, addrs, names):
    """Create Tmc2209 instances (does NOT do any UART communication)."""
    drivers = []
    for addr, axis_name in zip(addrs, names):
        com = TmcComUart(args.serial, args.baud)
        tmc = Tmc2209(
            tmc_ec=None,
            tmc_mc=None,
            tmc_com=com,
            driver_address=addr,
            loglevel=Loglevel.INFO,
            logprefix=f"TMC2209-{axis_name}",
        )
        drivers.append((tmc, axis_name))
    return drivers


def configure_all(drivers, h):
    """Configure all drivers. Returns True if all succeeded."""
    for tmc, axis_name in drivers:
        try:
            configure_driver(tmc, h, axis_name)
        except Exception as e:
            log.error("%s: configuration failed: %s", axis_name, e)
            return False
    return True


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(name)s: %(message)s",
    )

    args = parse_args()

    addrs = [int(a) for a in args.addr.split(",")]
    if args.names:
        names = args.names.split(",")
    else:
        names = [str(a) for a in addrs]

    if len(names) != len(addrs):
        log.error("Number of --names must match number of --addr")
        sys.exit(1)

    # Create HAL component and pins — no UART yet, drivers may be unpowered
    h = hal.component(args.name)
    create_hal_pins(h, names)
    h.ready()
    log.info("Component '%s' ready, waiting for 'enable' pin", args.name)

    drivers = init_drivers(args, addrs, names)
    was_enabled = False

    try:
        while True:
            enabled = h["enable"]

            if enabled and not was_enabled:
                # Rising edge: drivers just got power, configure them.
                # Give the TMC2209 a moment to finish its power-on reset.
                time.sleep(0.1)
                ok = configure_all(drivers, h)
                h["configured"] = ok
                if ok:
                    log.info("All drivers configured")
                else:
                    log.error("Driver configuration failed, will retry on next enable cycle")

            elif not enabled and was_enabled:
                # Falling edge: drivers lost power, mark unconfigured
                h["configured"] = False
                log.info("Enable de-asserted, drivers unconfigured")

            was_enabled = enabled

            # Only poll status when drivers are powered and configured
            if enabled and h["configured"]:
                for tmc, axis_name in drivers:
                    poll_status(tmc, h, axis_name)

            time.sleep(POLL_INTERVAL_S)
    except KeyboardInterrupt:
        pass
    finally:
        for tmc, _ in drivers:
            tmc.deinit()
        log.info("Shutdown complete")


if __name__ == "__main__":
    main()
