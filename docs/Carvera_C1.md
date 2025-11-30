# Carvera C1 Hardware Reference

This document was produced by analyzing
the [community firmware](https://github.com/Carvera-Community/Carvera_Community_Firmware) code.

## MCU

LPC1768 (ARM Cortex-M3)

| Parameter               | Value                     |
|-------------------------|---------------------------|
| Core Clock              | 100 MHz                   |
| Crystal                 | 12 MHz                    |
| TIMER0                  | Step ticker (priority 2)  |
| TIMER1                  | Unstep timer (priority 1) |
| TIMER2                  | Slow ticker (priority 4)  |
| TIMER3                  | Reserved (priority 4)     |
| PendSV                  | Priority 3                |
| ADC                     | Priority 5                |
| USB                     | Priority 5                |
| UART0-3                 | Priority 5                |
| Base Stepping Frequency | 100 kHz (default)         |
| Step Pulse Width        | 1 µs (default)            |

## Communication Interfaces

### I2C0 (Hardcoded)

| Function | Pin   |
|----------|-------|
| SDA0     | P0.27 |
| SCL0     | P0.28 |

- Frequency: 200 kHz

### I2C Devices

| Address   | Device                                     |
|-----------|--------------------------------------------|
| 0xA0/0xA1 | 24LC32A EEPROM (4KB, 128 pages × 32 bytes) |

### SSP0 - SD Card (Directly wired)

| Function | Pin   |
|----------|-------|
| MOSI0    | P0.18 |
| MISO0    | P0.17 |
| SCK0     | P0.15 |
| SSEL0    | P0.16 |

- Frequency: 12 MHz

### SSP1 - WiFi Module (M8266)

| Function  | Pin          |
|-----------|--------------|
| MOSI1     | P0.9         |
| MISO1     | P0.8         |
| SCK1      | P0.7         |
| SSEL1     | P0.6         |
| nRESET    | P2.10 (GPIO) |
| Interrupt | P2.11 (GPIO) |

- Initial frequency: 12 MHz (96 MHz / 8)
- Uses GPDMA for transfers

### UART0 - Wireless Probe (2.4GHz ZigBee module)

| Function | Pin  |
|----------|------|
| TXD0     | P0.2 |
| RXD0     | P0.3 |

- Baud rate: 115200 (configurable via `uart.baud_rate`)
- Communicates with wireless probe for voltage reporting, pairing, address setting

### UART3 (Serial Console)

| Function | Pin  |
|----------|------|
| TXD3     | P2.8 |
| RXD3     | P2.9 |

- Baud rate: 115200

### USB

| Function   | Pin   |
|------------|-------|
| USB Enable | P1.19 |

- Native USB device on LPC1768
- USB Serial (CDC)
- USB Mass Storage (SD card, disabled by default)

## Stepper Motors

| Axis        | Step Pin | Dir Pin | Enable Pin | Steps/mm           | Max Travel |
|-------------|----------|---------|------------|--------------------|------------|
| X (alpha)   | 1.28     | 1.29    | nc         | 200                | 500 mm     |
| Y (beta)    | 1.26     | 1.27    | nc         | 200                | 380 mm     |
| Z (gamma)   | 1.24     | 1.25    | nc         | 200                | 150 mm     |
| A (delta)   | 1.18     | 1.20!   | 3.26       | 26.667 (steps/deg) | 380°       |
| B (epsilon) | 1.21     | 1.23    | 1.30       | 43200              | -          |

## Motor Alarm Pins

| Axis    | Pin  | Inverted |
|---------|------|----------|
| X       | 0.1  | yes      |
| Y       | 0.0  | yes      |
| Z       | 3.25 | yes      |
| Spindle | 0.19 | no       |

## Endstops

| Axis | Min Pin | Max Pin | Homing Direction | Retract |
|------|---------|---------|------------------|---------|
| X    | 0.24^   | 0.25^   | home_to_max      | 2 mm    |
| Y    | 1.1^    | 1.4^    | home_to_max      | 2 mm    |
| Z    | -       | 1.8^    | home_to_max      | 2 mm    |
| A    | 0.21!^  | 0.21!^  | home_to_min      | 0.5 mm  |

## Digital Inputs

| Function                | Pin  | Pull-up      | Inverted |
|-------------------------|------|--------------|----------|
| E-Stop                  | 0.26 | yes          | yes      |
| Cover/Lid               | 1.9  | yes          | no       |
| Main Button             | 1.16 | yes          | no       |
| Probe (wireless)        | 2.6  | voltage mode | no       |
| Tool Setter (calibrate) | 0.5  | yes          | no       |

## Digital Outputs

| Function      | Pin  |
|---------------|------|
| Work Light    | 2.0  |
| Probe Charger | 0.23 |
| Tool Sensor   | 1.22 |
| Air Valve     | 0.11 |
| 12V Power     | 0.22 |
| 24V Power     | 0.10 |
| A-axis Enable | 3.26 |
| B-axis Enable | 1.30 |

## PWM Outputs

| Function       | Pin   | Peripheral | Period          |
|----------------|-------|------------|-----------------|
| Spindle        | P2.5  | PWM1.6     | 1000 µs (1 kHz) |
| Spindle Fan    | P2.1  | PWM1.2     | -               |
| Vacuum         | P2.3  | PWM1.4     | -               |
| Vacuum Digital | P2.13 | GPIO       | -               |
| EXT Port       | P2.2  | PWM1.3     | -               |

## ADC Inputs

| Function           | Pin   | Peripheral | Thermistor        |
|--------------------|-------|------------|-------------------|
| Spindle Thermistor | P1.31 | AD0.5      | 100kΩ NTC, β=3950 |

## Spindle

| Parameter          | Value         |
|--------------------|---------------|
| PWM Pin            | P2.5 (PWM1.6) |
| Encoder Pin        | P2.7 (CAP0.1) |
| Alarm Pin          | P0.19 (GPIO)  |
| Pulses/Rev         | 12            |
| Acc Ratio          | 1.635         |
| PWM Period         | 1000 µs       |
| Default RPM        | 10000         |
| Max Temp           | 60°C          |
| Delay after on/off | 3.0 s         |

## Main Button

| Function  | Pin   |
|-----------|-------|
| Button    | 1.16^ |
| LED Red   | 1.10  |
| LED Green | 1.15  |
| LED Blue  | 1.14  |

- Poll frequency: 20 Hz
- Long press time: 3000 ms

## Status LEDs (on board)

| LED    | Pin   |
|--------|-------|
| LED 0  | P4.29 |
| LED 1  | P4.28 |
| LED 2  | P0.4  |
| LED 3  | P1.17 |
| Buzzer | P1.14 |

## Work Envelope

| Axis | Min (mm) | Max (mm) | Max Travel | Fast Homing | Slow Homing |
|------|----------|----------|------------|-------------|-------------|
| X    | -371     | 0        | 500 mm     | 15 mm/s     | 3 mm/s      |
| Y    | -250     | 0        | 380 mm     | 15 mm/s     | 3 mm/s      |
| Z    | -135     | 0        | 150 mm     | 10 mm/s     | 3 mm/s      |
| A    | -0.5°    | 0°       | 380°       | 50 mm/s     | 10 mm/s     |

## Default Coordinates (MCS)

| Parameter        | X        | Y        | Z      |
|------------------|----------|----------|--------|
| Anchor 1         | -360.158 | -234.568 | -      |
| Anchor 2 Offset  | +90.0    | +45.0    | -      |
| Tool Rack Offset | +356     | 0        | -112.5 |
| Tool Setter      | -4.158   | -42.568  | -157   |
| Rotation Offset  | +3.5     | +37.5    | +23    |
| Clearance        | -75      | -3       | -3     |
| Rotation Width   | 100      | -        | -      |

## ATC

| Function       | Pin   |
|----------------|-------|
| Homing Endstop | 1.0^  |
| Tool Detector  | 0.20^ |

| Parameter         | Value                    |
|-------------------|--------------------------|
| Tool Slots        | 6 (8 with CE1 expansion) |
| Tool Spacing      | 30 mm (25 mm with CE1)   |
| Homing Max Travel | 8 mm                     |
| Homing Rate       | 0.4 mm/s                 |
| Action Rate       | 0.25 mm/s                |

## WiFi Module (M8266)

| Parameter     | Value |
|---------------|-------|
| TCP Port      | 2222  |
| UDP Send Port | 3333  |
| UDP Recv Port | 4444  |
| TCP Timeout   | 10 s  |

## Laser Module (Optional)

| Function | Pin   | Peripheral |
|----------|-------|------------|
| PWM      | P2.4  | PWM1.5     |
| Enable   | P2.12 | GPIO       |

## Probe

| Parameter                 | Value  |
|---------------------------|--------|
| Tip Diameter              | 1.6 mm |
| Debounce                  | 1 ms   |
| Calibration Safety Margin | 0.1 mm |

## Wireless Probe

| Parameter                    | Value                                            |
|------------------------------|--------------------------------------------------|
| Interface                    | UART0 (P0.2/P0.3) via 2.4GHz ZigBee              |
| Charger Pin                  | 0.23                                             |
| Min Voltage (start charging) | 3.6 V                                            |
| Max Voltage (stop charging)  | 4.1 V                                            |
| M-codes                      | M470 (set address), M471 (pairing), M472 (laser) |

