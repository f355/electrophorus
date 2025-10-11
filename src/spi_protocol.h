#ifndef SPIDATA_H
#define SPIDATA_H

#include "machine_definitions.h"

#define PRU_DATA 0x64617461   // "data" SPI payload
#define PRU_READ 0x72656164   // "read" SPI payload
#define PRU_WRITE 0x77726974  // "writ" SPI payload

// SPI configuration
#define SPI_BUF_SIZE 56  // maximum of rx/tx sizes

#define FIXED_POINT 32
#define FIXED_ONE (1LL << FIXED_POINT)

#pragma pack(push, 2)

// struct for LinuxCNC -> PRU communication
// byte size: 56
typedef union {
  uint8_t buffer[SPI_BUF_SIZE];
  struct {
    int32_t header;
    volatile float stepgen_freq_command[STEPGENS];
    int32_t output_vars[OUTPUT_VARS];
    uint8_t stepgen_enable_mask;
    uint16_t outputs;
  };
} linuxCncState_t;

// struct for PRU -> LinuxCNC communication
// byte size: 50
typedef union {
  uint8_t buffer[SPI_BUF_SIZE];
  struct {
    int32_t header;
    int64_t stepgen_feedback[STEPGENS];
    int32_t input_vars[INPUT_VARS];
    uint16_t inputs;
  };
} pruState_t;

#pragma pack(pop)

#endif
