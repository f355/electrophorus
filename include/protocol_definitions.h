#ifndef PROTOCOL_DEFINITIONS_H
#define PROTOCOL_DEFINITIONS_H

#include <stdint.h>

#include "machine_definitions.h"

// Wire format is little-endian.

// Magics
#define PRU_DATA 0x61746164u   // ASCII bytes on wire: 'd','a','t','a'
#define PRU_WRITE 0x21636e63u  // ASCII bytes on wire: 'c','n','c','!'

#define FIXED_POINT 32
#define FIXED_ONE (1LL << FIXED_POINT)

// Deterministic 4-byte packing ensures consistent layout across MCU and host
#pragma pack(push, 4)

// PRU -> Host
typedef struct {
  uint32_t header;  // PRU_DATA
  int64_t stepgen_feedback[STEPGENS];
  int32_t input_vars[INPUT_VARS];
  uint16_t inputs;
  uint16_t timestamp;  // linuxcnc-provided us timestamp echo
  uint32_t crc;
} pruState_t;

// Host -> PRU
typedef struct {
  uint32_t header;  // PRU_WRITE
  float stepgen_freq_command[STEPGENS];
  int32_t output_vars[OUTPUT_VARS];
  uint8_t stepgen_enable_mask;
  uint16_t outputs;
  uint16_t timestamp;  // linuxcnc-provided us timestamp (mod 65536)
  uint32_t crc;
} linuxCncState_t;

#pragma pack(pop)

#define PRU_TO_HOST_FRAME_BYTES ((uint32_t)sizeof(pruState_t))
#define HOST_TO_PRU_FRAME_BYTES ((uint32_t)sizeof(linuxCncState_t))

#endif  // PROTOCOL_DEFINITIONS_H
