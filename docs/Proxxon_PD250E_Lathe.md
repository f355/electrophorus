# Proxxon PD 250/E Lathe Conversion

CNC conversion using Carvera C1 controller board running electrophorus firmware.

## Hardware Overview

- **Controller**: Carvera C1 board (LPC1768)
- **Lathe**: Proxxon PD 250/E
- **Stepper Drivers**: Onboard TMC2209 (×2) in step/dir mode
- **External**: Raspberry Pi Pico 2 for glass scales and spindle encoder

## Axis Mapping

| Lathe Axis      | C1 Axis     | Driver  | Step  | Dir   | Enable |
|-----------------|-------------|---------|-------|-------|--------|
| X (cross slide) | A (delta)   | TMC2209 | P1.18 | P1.20 | P3.26  |
| Z (carriage)    | B (epsilon) | TMC2209 | P1.21 | P1.23 | P1.30  |

Note: P1.20 and P1.23 are shared with QEI (MCI0/MCI1), so hardware quadrature decoding on LPC1768 is not available when
using onboard drivers.

## Home Switches

| Axis   | Pin   | Pull-up | Notes                    |
|--------|-------|---------|--------------------------|
| X Home | P0.24 | yes     | Repurposed from C1 X-min |
| Z Home | P0.25 | yes     | Repurposed from C1 X-max |

Alternative options:

- P1.1 (C1 Y-min)
- P1.4 (C1 Y-max)
- P1.8 (C1 Z-max)
- P0.21 (C1 A-axis home, active low)

### Sensor Selection

Inductive proximity sensors recommended - no contact, immune to chips/coolant.

| Part Number   | Type               | Sensing Dist | Voltage | Notes                                  |
|---------------|--------------------|--------------|---------|----------------------------------------|
| LJ12A3-4-Z/BX | M12 barrel, NPN NO | 4mm          | 6-36V   | Needs level shifter for 3.3V logic     |
| LJ12A3-4-Z/BY | M12 barrel, NPN NC | 4mm          | 6-36V   | Needs level shifter for 3.3V logic     |
| SN04-N        | M4 barrel, NPN NO  | ~1mm         | 5V      | May work direct, smaller sensing range |

Search AliExpress for "LJ12A3-4-Z/BX NPN" or "inductive proximity sensor M12 NPN".

### Mounting Considerations

The sensor creates an alternating magnetic field from its face. Nearby ferrous metal (cast iron bed) will dampen the
oscillator and reduce sensing distance or prevent operation.

**Do:**

- Mount sensor face proud of any ferrous bracket by at least the sensing distance
- Use aluminum or plastic bracket to isolate from cast iron bed
- Point sensor face at a machined surface on carriage for repeatability

**Don't:**

- Recess sensor flush into a cast iron block
- Mount sensor face parallel and close to the bed surface

A simple aluminum L-bracket bolted to the bed works well.

### Z Axis Homing Strategy

Home switch mounted mid-bed, triggered by a flag/protrusion on the carriage:

- Avoids tailstock area (can leave tailstock in place)
- Avoids chuck area (no risk of crash during homing)
- Always approach from same direction for repeatability (set HOME_SEARCH_VEL sign in LinuxCNC)
- Set HOME_OFFSET so Z0 ends up at chuck face or other useful reference

## Spindle Control

Using WS55-220 BLDC driver.

| Function     | Pin   | Peripheral | WS55-220 Pin | Notes                      |
|--------------|-------|------------|--------------|----------------------------|
| Speed (0-5V) | P2.5  | PWM1.6     | SV           | PWM + RC filter for analog |
| Enable       | P2.0  | GPIO       | EN           | Active high                |
| Direction    | P0.23 | GPIO       | FR           | Low=CW, High=CCW           |

### WS55-220 Wiring

| WS55-220 Terminal    | Connection                               |
|----------------------|------------------------------------------|
| SV (Speed Voltage)   | P2.5 via RC filter (10kΩ + 1µF) for 0-5V |
| EN (Enable)          | P2.0 (3.3V logic OK)                     |
| FR (Forward/Reverse) | P0.23                                    |
| GND                  | Common ground with C1 board              |
| +5V                  | Can power from C1 5V rail                |
| ACN/ACL              | Mains input                              |
| U/V/W                | BLDC motor phases                        |

### Speed Control

The WS55-220 expects 0-5V analog on SV pin. Use PWM with RC low-pass filter:

- PWM frequency: 20-50 kHz recommended
- RC filter: 1kΩ + 100nF gives ~1.6 kHz cutoff, good response with minimal ripple at 20kHz+
- 0% duty = 0V = min RPM, 100% duty = 3.3V ≈ 66% max speed

For full 0-5V range, add a simple op-amp buffer/scaler, or accept reduced speed range.

## Tool Setter (ETS)

| Function    | Pin  | Pull-up |
|-------------|------|---------|
| Tool Setter | P0.5 | yes     |

Uses existing C1 tool setter input. Wire to touch-off probe or tool length sensor.

## E-Stop

Hardware e-stop circuit that cuts power independent of software.

### Architecture

```
[E-Stop Button(s)] --NC series--> [Safety Relay] ---> [Contactor Coil]
                                        |
                                        +--> [LPC1768 P0.26] (status feedback)

Contactor controls:
  - 24V rail to stepper drivers
  - WS55-220 enable or power
```

### Components

| Part          | Budget                     | Brand-name                          | Notes                                        |
|---------------|----------------------------|-------------------------------------|----------------------------------------------|
| E-Stop Button | XB2-BS542, LAY38-11ZS      | Schneider XB4-BS542, ABB CE4T-10R01 | NC contacts, twist-to-release, mushroom head |
| Power Relay   | JQX-13F 24VDC, HH52P 24VDC | Omron G2R-1-E 24DC, Finder 40.31    | 10A+ contact, 24V coil, DIN rail mount       |
| Contactor     | CJX2-0910                  | Schneider LC1D09, ABB AF09          | 24V coil, 9A+ rated                          |
| Safety Relay  | SLA-A-24VDC-SL-2C1NO1NC    | Pilz PNOZ s4, Omron G9SA-301        | Force-guided contacts, monitors e-stop loop  |

Compact alternatives (no DIN rail):

| Part       | Model          | Notes                                              |
|------------|----------------|----------------------------------------------------|
| Cube Relay | SRD-24VDC-SL-C | ~$1, 10A, PCB pins or socket, 19×15×15mm           |
| Cube Relay | SLA-24VDC-SL-A | ~$2, 30A, PCB/socket, beefier for motor power      |
| SSR        | SSR-25DA       | ~$5, 25A, 3-32VDC control, panel mount with screws |

The cube relays fit in cheap sockets (search "SRD relay socket" or "5-pin relay socket PCB"). SSRs need a heatsink if
running near rated current but are silent and compact.

Search AliExpress for "CJX2-0910 24V", "JQX-13F 24VDC relay", "XB2-BS542", "SRD-24VDC-SL-C socket".

### Wiring

1. **E-stop button**: NC contact controls the relay, NO contact signals the MCU.

2. **Relay**: Controls 24V power to TMC2209 drivers and WS55-220 EN pin.

3. **Reset sequence**:
    - Release e-stop button (twist or pull)
    - Relay re-energizes, power restored
    - LinuxCNC requires software reset acknowledgment

### Schematic

With a single SPDT button (NC/C/NO), use C-NC for the relay and take status from a spare relay contact. Use a relay with 2 NO contacts (DPDT like JQX-13F or HH52P).

```
   Button controls relay:

   24V -----[ Button C-NC ]----[ Relay Coil ]----- GND


   Relay NO1 switches motor power:

   24V_SUPPLY -----[ Relay NO1 ]----- 24V_MOTORS


   Relay NO2 signals MCU:

                  +---[ Relay NO2 ]--- 3.3V
                  |
               P0.26
                  |
                [10k]
                  |
                 GND
```

Normal: button released, C-NC closed, relay energized, NO1+NO2 closed, motors powered, P0.26 high.

E-stop: button pressed, C-NC open, relay drops, NO1+NO2 open, motors off, P0.26 pulled low via 10k.

### Notes

- Use NC (normally closed) contacts so a broken wire triggers e-stop
- The LPC1768 input is status only - actual power cut is hardware
- For the WS55-220, cutting EN is sufficient; cutting DC bus is more aggressive but adds inrush on restart

## External Pico 2 Interface

The Pico 2 handles high-frequency counting that would otherwise burden the LPC1768:

- Glass scale X (quadrature)
- Glass scale Z (quadrature)
- Spindle encoder (quadrature, mounted on spindle shaft for accurate position/velocity)

| Function | Pico Pin | LPC1768 Pin | Notes                                               |
|----------|----------|-------------|-----------------------------------------------------|
| SPI MOSI | GPxx     | P0.9 (SSP1) | Data to Pico                                        |
| SPI MISO | GPxx     | P0.8 (SSP1) | Data from Pico                                      |
| SPI SCK  | GPxx     | P0.7 (SSP1) | Clock                                               |
| SPI CS   | GPxx     | P0.6 (SSP1) | Directly drive via GPIO, or use existing nRESET/IRQ |

Alternative: Use I2C0 (P0.27 SDA, P0.28 SCL) if SPI conflicts with WiFi module.

### Pico 2 Responsibilities

1. **Glass Scales**: Hardware quadrature decoding via PIO, 32-bit position counters
2. **Spindle Encoder**: RPM calculation, position for threading/rigid tapping
3. **Protocol**: Respond to SPI commands with current positions/velocities

## Available Spare Pins

These C1 pins are unused in lathe config and available for expansion:

| Pin   | Original Function  | Notes       |
|-------|--------------------|-------------|
| P2.6  | Wireless Probe     | GPIO input  |
| P0.23 | Probe Charger      | GPIO output |
| P1.22 | Tool Sensor        | GPIO output |
| P0.11 | Air Valve          | GPIO output |
| P2.1  | Spindle Fan PWM    | PWM1.2      |
| P2.3  | Vacuum PWM         | PWM1.4      |
| P2.2  | EXT Port PWM       | PWM1.3      |
| P2.13 | Vacuum Digital     | GPIO output |
| P1.31 | Spindle Thermistor | AD0.5       |

## Power Rails

| Rail | Control Pin | Notes           |
|------|-------------|-----------------|
| 12V  | P0.22       | For accessories |
| 24V  | P0.10       | Stepper power   |

## Firmware Configuration

In `src/machine_config.cc`, configure for lathe:

```cpp
vector<Module*> MachineModules() {
  return {
      // X = cross slide (was A-axis)
      new Stepgen(0, new Pin(1, 18), new Pin(1, 20)),
      // Z = carriage (was B-axis)  
      new Stepgen(1, new Pin(1, 21), new Pin(1, 23)),

      new EStop((new Pin(0, 26))->Invert()),

      // Home switches
      new DigitalIns(2, (Pin*[]){
          (new Pin(0, 24))->PullUp(),  // X home
          (new Pin(0, 25))->PullUp(),  // Z home
      }),

      // Tool setter, spindle on
      new DigitalOuts(2, (Pin*[]){
          new Pin(2, 0),   // spindle enable (accent light repurposed)
      }),

      // Spindle PWM
      new PWM(1, 0, new Pin(2, 5)),

      // Pico 2 interface - positions come via SPI as input_vars
  };
}
```

## LinuxCNC Configuration Notes

- Lathe mode: `LATHE = 1` in INI
- Axis: X (diameter mode), Z
- Spindle: synchronized for threading via Pico 2 encoder feedback
- Tool table: lathe tool orientations

## TODO

- [ ] Determine glass scale resolution/protocol
- [ ] Design Pico 2 firmware and SPI protocol
- [ ] Verify TMC2209 current settings for lathe motors
- [ ] Spindle encoder PPR for PD 250/E
- [ ] Mechanical mounting for home switches

