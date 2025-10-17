#ifndef SPIDATA_H
#define SPIDATA_H

#include "machine_definitions.h"

#define PRU_READ 0x72656164      // "read" SPI command, LinuxCNC wants to read PRU state
#define PRU_WRITE 0x77726974     // "writ" SPI command, LinuxCNC wants to write its state
#define PRU_INIT_PKT 0x696E6974  // "init" initial value for PRU packet counter

// SPI configuration
#define SPI_BUF_SIZE 40  // maximum of rx/tx sizes

#define FIXED_POINT 32
#define FIXED_ONE (1LL << FIXED_POINT)

#pragma pack(push, 4)

// struct for LinuxCNC -> PRU communication
// byte size w/o buffer: 40
typedef union {
  uint8_t buffer[SPI_BUF_SIZE];
  struct {
    int32_t command;
    uint32_t steps_per_tick_cmd[STEPGENS];  // Q0.32 steps per base tick (magnitude)
    int32_t output_vars[OUTPUT_VARS];
    uint8_t stepgen_dir_mask;  // bit i = forward
    uint16_t outputs;
  };
} linuxCncState_t;

// struct for PRU -> LinuxCNC communication
// byte size w/o buffer: 36
typedef union {
  uint8_t buffer[SPI_BUF_SIZE];
  struct {
    uint32_t packet_counter;  // increments on each PRU_READ; initialized to PRU_INIT_PKT
    int32_t input_vars[INPUT_VARS];
    uint16_t inputs;
    int32_t stepgen_feedback[STEPGENS];
  };
} pruState_t;

#pragma pack(pop)

#endif
