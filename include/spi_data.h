#ifndef SPIDATA_H
#define SPIDATA_H

#include "machine_definitions.h"

#define PRU_DATA 0x64617461    // "data" SPI payload
#define PRU_READ 0x72656164    // "read" SPI payload
#define PRU_WRITE 0x77726974   // "writ" SPI payload
#define PRU_CONFIG 0x636f6e66  // "conf" SPI payload - config command

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
    union {
      struct {  // Normal data payload (PRU_WRITE)
        volatile float stepgen_position_cmd[STEPGENS];
        int32_t output_vars[OUTPUT_VARS];
        uint8_t stepgen_enable_mask;
        uint16_t outputs;
      };
      struct {                             // Config payload (PRU_CONFIG)
        float stepgen_maxvel[STEPGENS];    // steps/sec
        float stepgen_maxaccel[STEPGENS];  // steps/sec^2
        float stepgen_init_pos[STEPGENS];  // initial position in steps
      };
    };
  };
} linuxCncData_t;

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
} pruData_t;

#pragma pack(pop)

#endif
