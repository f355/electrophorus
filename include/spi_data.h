#ifndef SPIDATA_H
#define SPIDATA_H

#include "machine_definitions.h"

#define PRU_DATA 0x64617461   // "data" SPI payload
#define PRU_READ 0x72656164   // "read" SPI payload
#define PRU_WRITE 0x77726974  // "writ" SPI payload
#define PRU_CONF 0x636F6E66   // "conf" SPI payload

// SPI configuration
#define SPI_BUF_SIZE 56  // maximum of rx/tx sizes

typedef int64_t fixp_t;  // 32.32 fixed point type to somewhat distinguish it from integers
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
      struct {
        // write payload
        volatile fixp_t stepper_pos_command[STEPGENS];  // position commands for steppers in machine units (mu)
        uint8_t stepper_enable_mask;                    // bitmask for enabled steppers
        int32_t output_vars[OUTPUT_VARS];               // output variables (PWM duty etc.)
        uint16_t outputs;                               // output GPIO pin states, bitmask
      };
      struct {
        // config payload
        uint32_t stepper_init_position[STEPGENS];   // initial stepper position, 24.8 fixed-point
        uint32_t stepper_max_accel[STEPGENS];       // maximum stepper acceleration in mu/s/s, 24.8 fixed-point
        uint32_t stepper_position_scale[STEPGENS];  // number of steps per mu, 24.8 fixed-point
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
    fixp_t stepper_position_fb[STEPGENS];  // position feedback in mu
    int32_t input_vars[INPUT_VARS];        // input variables (ADC values etc.)
    uint16_t inputs;                       // input GPIO pin states, bitmask
  };
} pruData_t;

#pragma pack(pop)

#endif
