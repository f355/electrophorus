#ifndef SPIDATA_H
#define SPIDATA_H

#include "machine_definitions.h"

#define PRU_DATA 0x64617461   // "data" SPI payload
#define PRU_READ 0x72656164   // "read" SPI payload
#define PRU_WRITE 0x77726974  // "writ" SPI payload
#define PRU_ESTOP 0x65737470  // "estp" SPI payload

// SPI configuration
#define SPI_BUF_SIZE 40  // maximum of rx/tx sizes

#pragma pack(push, 2)

// struct for LinuxCNC -> PRU communication
// byte size: 40
typedef union {
  struct {
    uint8_t buffer[SPI_BUF_SIZE];
  };
  struct {
    int32_t header;
    volatile int32_t joint_freq_command[JOINTS];
    int32_t output_vars[OUTPUT_VARS];
    uint8_t joint_enable;
    uint16_t outputs;
  };
} linuxCncData_t;

// struct for PRU -> LinuxCNC communication
// byte size: 34
typedef union {
  struct {
    uint8_t buffer[SPI_BUF_SIZE];
  };
  struct {
    int32_t header;
    int32_t joint_feedback[JOINTS];
    int32_t input_vars[INPUT_VARS];
    uint16_t inputs;
  };
} pruData_t;

#pragma pack(pop)

#endif
