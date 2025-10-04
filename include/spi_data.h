#ifndef SPIDATA_H
#define SPIDATA_H

#include "machine_definitions.h"

#define PRU_DATA 0x64617461   // "data" SPI payload
#define PRU_READ 0x72656164   // "read" SPI payload
#define PRU_WRITE 0x77726974  // "writ" SPI payload

#define SPI_BUF_SIZE 56  // maximum of rx/tx sizes

#define FIXED_POINT 32
#define FIXED_ONE (1LL << FIXED_POINT)

// number of bytes used for CRC calculation (exclude the crc32 field itself)
#define LINUXCNC_CRC_LEN (4 + (STEPGENS * 4) + (OUTPUT_VARS * 4) + 1 + 1 + 2)
#define PRU_CRC_LEN (4 + (STEPGENS * 8) + (INPUT_VARS * 4) + 2 + 2) /* +2 pad before crc32 under pack(4) */

#pragma pack(push, 4)

// struct for LinuxCNC -> PRU communication
typedef union {
  uint8_t buffer[SPI_BUF_SIZE];
  struct {
    int32_t header;
    volatile float stepgen_freq_command[STEPGENS];
    int32_t output_vars[OUTPUT_VARS];
    uint8_t stepgen_enable_mask;
    uint16_t outputs;
    uint32_t crc32;  // CRC over first LINUXCNC_CRC_LEN bytes
  };
} linuxCncData_t;

// struct for PRU -> LinuxCNC communication
typedef union {
  uint8_t buffer[SPI_BUF_SIZE];
  struct {
    int32_t header;
    int64_t stepgen_feedback[STEPGENS];
    int32_t input_vars[INPUT_VARS];
    uint16_t inputs;
    uint32_t crc32;  // CRC over first PRU_CRC_LEN bytes
  };
} pruData_t;

#pragma pack(pop)

#endif
