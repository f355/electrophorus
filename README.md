# electrophorus

**WARNING: The author(s) assume absolutely no responsibility for any damage and/or disappointment that might occur as a
result of following the instructions below. This project is currently still is heavily work in progress (almost nothing
works!) and would work only on Carvera Air. DO NOT TRY THIS AT HOME unless you understand what you're doing and are
willing to deal with the issues and fix the bugs yourself (I might help with that, though).**

Electrophorus is a project that allows you to convert a Carvera-family desktop CNC milling machine
by [Makera Inc.](https://www.makera.com/)
to use [LinuxCNC](https://linuxcnc.org/) as a controller with the help of
a [Raspberry Pi](https://www.raspberrypi.com/).

It works by offloading the realtime functions (pulsing the motor drivers) to the Carvera board/firmware, while the
computation-heavy parts (G code parsing, trajectory planning, motion control, etc.) are handled by LinuxCNC running on
the Raspberry Pi. If you're familiar with LinuxCNC concepts, the Carvera board acts as a Programmable Realtime Unit
(PRU) on BeagleBone or a limited version of a Mesa card.

I (@f355) don't have the big-brother Carvera (C1), so for now it is supposed to only work (or not) with
Carvera Air (CA1).

The project has started as a fork of the fantastic [Remora](https://github.com/scottalford75/Remora) project. The Remora
authors [say they "dont ...not support"](https://github.com/scottalford75/Remora/issues/78#issuecomment-2584956914)
LPC1768-based boards, so this is a hard-fork, the changes are not intended to be upstreamed, and the code has been
pretty much rewritten.

## Wiring (Carvera Air)

#### USB

Just connect the Raspberry Pi to the Carvera Air using a USB-C-to-USB-A cable. It is enough if you're not planning to do
any development/debugging and just want to use the machine.

#### Connector J3 - UART

Totally optional and not needed for normal operation. Used to see the firmware console output, mostly for
debugging/informational purposes.

It's a 4-pin JST-XH connector right below the MCU, where the unused CAM cable is plugged in.

Pins are numbered top-to-bottom, according to the silkscreen.

**NOTE: the board RX should be connected to the RPi TX and vice versa!**

| J3 Pin# | Signal | RPi pin# |
|---------|--------|----------|
| 1       | GND    | 6        |
| 2       | 3V3    | NC       |
| 3       | RX/TX  | 10       |
| 4       | TX/RX  | 8        |

#### Connector J12 - SWD/Reset

Totally optional and not needed for normal operation, but very convenient for development, it can be used for
flashing, resetting and debugging the firmware.

It's a 5-pin Dupont pin header below and to the right of the MCU.

Pins are numbered bottom-to-top, according to the silkscreen.

| J12 Pin# | Signal | RPi pin# | RPi GPIO# |
|----------|--------|----------|-----------|
| 1        | 3V3    | NC       | N/A       |
| 2        | SWDIO  | 22       | 25        |
| 3        | GND    | 20       | N/A       |
| 4        | SWDCLK | 18       | 24        |
| 5        | RESET  | 16       | 23        |

### Building the firmware

Follow the [official Mbed CE instructions](https://mbed-ce.dev/getting-started/toolchain-install/). Use this repo
instead of the `mbed-ce-hello-world`, `Release` build type and `LPC1768` target.

If you're doing this on the Raspberry Pi itself, and the SWD port is wired up, you can flash the firmware with
`sudo ninja flash-electrophorus`.

Instead of flashing the firmware through SWD, you can rename `build/electrophorus.bin` to `firmware.bin` and put it in
the root folder of the SD card as usual. The firmware is not using the SD card at all, so you can leave the rest of the
files on it.

On the first flash, you can also use Carvera Controller to flash the firmware as usual, just upload
`build/electrophorus.bin`.

To go back to the [stock](https://github.com/MakeraInc/CarveraFirmware/releases)
or [community](https://github.com/Carvera-Community/Carvera_Community_Firmware/releases) firmware you need to download
it, rename it to `firmware.bin` and put it on the SD card. There's currently no mechanism to do this without
removing the SD card.

### Configuring Raspberry Pi

1. Use [Raspberry Pi Imager](https://www.raspberrypi.com/software/) to prepare the SD card as usual (Lite variant is
   recommended and the only one tested so far). Boot the Raspberry Pi from the SD card and connect to the network.
2. From the terminal application or an SSH session:

```shell
# fetch package repository index
sudo apt update

# install git and clone the repository
sudo apt install git
git clone https://github.com/f355/electrophorus.git
cd electrophorus

# install dependencies and configure things - this will take several minutes to complete.
# If it asks questions, say yes, even if it looks somewhat scary.
./install.sh

# reboot the Raspberry Pi for the changes to take effect
sudo reboot
```

### Configuring LinuxCNC

Coming soon, carefuling is in progress.

## Historical: Remora - the original README

The full documentation is at <https://remora-docs.readthedocs.io/en/latest/>
Note: Docs have not been updated for 1.0.0_rc

Remora is a free, opensource LinuxCNC component and Programmable Realtime Unit (PRU) firmware to allow LPC176x and
STM32F4 based controller boards to be used in conjuction with a Raspberry Pi to implement a LinuxCNC based CNC
controller.

Having a low cost and accessable hardware platform for LinuxCNC is important if we want to use LinuxCNC for 3D printing
for example. Having a controller box the size of the printer itself makes no sense in this applicatoin. A SoC based
single board computer is ideal in this application. Although developed for 3D Printing, Remora (and LinuxCNC) is highly
flexible and configurable for other CNC applications.

Remora has been in use amd development since 2017. Starting on Raspberry Pi 3B and 3B+ eventhough at the time it was
percieved that the Raspberry Pi was not a viable hardware for LinuxCNC.

With the release of the RPi 4 the LinuxCNC community now supports the hardware, with LinuxCNC and Preempt-RT Kernel
packages now available from the LinuxCNC repository. This now greatly simplifies the build of a Raspberry Pi based CNC
controller.
