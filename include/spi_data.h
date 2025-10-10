#ifndef SPIDATA_H
#define SPIDATA_H

#include <stddef.h>

#include "machine_definitions.h"

#define PRU_READ 0x72656164   // "read" command
#define PRU_WRITE 0x77726974  // "writ" command

#define FIXED_POINT 32
#define FIXED_ONE (1LL << FIXED_POINT)

#pragma pack(push, 4)

typedef struct {
  volatile float stepgen_freq_command[STEPGENS];
  int32_t output_vars[OUTPUT_VARS];
  uint8_t stepgen_enable_mask;
  uint16_t outputs;
  uint32_t crc32;  // CRC over first LINUXCNC_CRC_LEN bytes
} linuxCncState_t;

typedef struct {
  int64_t stepgen_feedback[STEPGENS];
  int32_t input_vars[INPUT_VARS];
  uint16_t inputs;
  uint32_t crc32;  // CRC over first PRU_CRC_LEN bytes
} pruState_t;

#pragma pack(pop)

#endif  // SPIDATA_H
