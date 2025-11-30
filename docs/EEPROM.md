# Carvera EEPROM Layout

This document was produced by analyzing
the [community firmware](https://github.com/Carvera-Community/Carvera_Community_Firmware) code.

Both Carvera machines use a **24LC32A** I2C EEPROM (32Kbit / 4KB) with 32-byte pages (128 pages total).

## Memory Map

| Start | End   | Region      | Pages  | Size | Description                                         |
|-------|-------|-------------|--------|------|-----------------------------------------------------|
| 0x000 | 0x01F | Unused      | 0      | 32   |                                                     |
| 0x020 | 0x107 | EEPROM_data | 1-7    | 232  | User/runtime data (with 2-byte header + 2-byte CRC) |
| 0x108 | 0x1FF | Unused      | 8-15   | 248  |                                                     |
| 0x200 | 0x207 | FACTORY_SET | 16     | 8    | Factory settings (with 2-byte header + 2-byte CRC)  |
| 0x208 | 0xFFF | Unused      | 17-127 | 3576 |                                                     |

## EEPROM_data Structure (pages 1-7)

| Address | Field                     | Type      | Size | Description                                                                     |
|---------|---------------------------|-----------|------|---------------------------------------------------------------------------------|
| 0x020   | header                    | uint16    | 2    | Magic bytes 0x5A 0xA5                                                           |
| 0x022   | TLO                       | float     | 4    | Tool Length Offset (Z component of current tool offset)                         |
| 0x026   | REFMZ                     | float     | 4    | Reference tool Z position in machine coords (used for tool length compensation) |
| 0x02A   | TOOLMZ                    | float     | 4    | Current tool Z position in machine coords (measured at tool setter)             |
| 0x02E   | reserve                   | float     | 4    | Reserved/unused                                                                 |
| 0x032   | TOOL                      | int       | 4    | Currently loaded tool number (0 = no tool, 999990+ = probe)                     |
| 0x036   | perm_vars[20]             | float[20] | 80   | Persistent G-code variables #501-#520                                           |
| 0x086   | probe_tool_not_calibrated | bool      | 1    | True if probe tool hasn't been calibrated                                       |
| 0x087   | (padding)                 | -         | 3    | Alignment padding                                                               |
| 0x08A   | current_wcs               | int       | 4    | Last active WCS (0=G54, 1=G55, ..., 5=G59)                                      |
| 0x08E   | WCScoord[6][4]            | float[24] | 96   | WCS offsets for G54-G59: [wcs][X,Y,Z,A]                                         |
| 0x0EE   | WCSrotation[6]            | float[6]  | 24   | WCS rotation angles for G54-G59 (degrees)                                       |
| 0x106   | crc                       | uint16    | 2    | CRC16-CCITT over 0x022-0x105                                                    |

## FACTORY_SET Structure (page 16)

| Address | Field        | Type   | Size | Description                             |
|---------|--------------|--------|------|-----------------------------------------|
| 0x200   | header       | uint16 | 2    | Magic bytes 0x5A 0xA5                   |
| 0x202   | MachineModel | char   | 1    | 1 = CARVERA (C1), 2 = CARVERA_AIR (CA1) |
| 0x203   | FuncSetting  | char   | 1    | Bit flags (see below)                   |
| 0x204   | reserve1     | char   | 1    | Reserved                                |
| 0x205   | reserve2     | char   | 1    | Reserved                                |
| 0x206   | crc          | uint16 | 2    | CRC16-CCITT over 0x202-0x205            |

### FuncSetting Bit Flags

| Bit | Mask | Name               | Description                                                                                                                           |
|-----|------|--------------------|---------------------------------------------------------------------------------------------------------------------------------------|
| 0   | 0x01 | A-axis home enable | Enables homing for the A (rotary) axis                                                                                                |
| 1   | 0x02 | C-axis home enable | Enables homing for the C axis (if present)                                                                                            |
| 2   | 0x04 | ATC enable         | Automatic Tool Changer enabled; affects status reporting, vacuum vs powerfan control, and ATC state machine. Always forced on for C1. |
| 3   | 0x08 | CE1 Expand         | CE1 expansion board present                                                                                                           |
| 4-7 |      | (unused)           | Reserved                                                                                                                              |

The default value is 0x04 (ATC enabled). For C1 machines, bit 2 is always forced on regardless of stored value.

## Notes

- All data regions use a 2-byte header (0x5A 0xA5) and 2-byte CRC16-CCITT for validation
- If FACTORY_SET validation fails on read, firmware defaults to MachineModel=1 (C1), FuncSetting=0x04
- EEPROM_data is written frequently during operation (WCS changes, tool changes, etc.)
- FACTORY_SET is typically only written at the factory or via `/sd/factory.ini` on boot

