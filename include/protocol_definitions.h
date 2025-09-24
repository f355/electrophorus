#ifndef PROTOCOL_DEFINITIONS_H
#define PROTOCOL_DEFINITIONS_H

#include <stdint.h>

#include "machine_definitions.h"

// Wire format is little-endian (ARM). If porting to a big-endian MCU/host, convert when
// accessing scalars via the buffer[] view.

#define PRU_DATA 0x61746164  // ASCII bytes on wire: 'd','a','t','a'
#define PRU_CONF 0x666e6f63  // ASCII bytes on wire: 'c','o','n','f'

#define FIXED_POINT 32
#define FIXED_ONE (1LL << FIXED_POINT)

// 62 so MCU→host replies fit into one FT232R USB IN packet:
// 64‑byte USB FS packet = 2 status bytes (FTDI) + 62 data. Lowest latency/jitter.
// Changing this is problematic:
//  - >62 spills into 2 USB packets → higher/variable latency
//  - <62 wastes bandwidth and may trigger latency‑timer effects
// Struct fields currently fit within 62; union sizeof may be 64 due to alignment.
// Always TX/RX exactly XFER_BUF_SIZE bytes.
#define XFER_BUF_SIZE 62

// struct for LinuxCNC -> PRU communication (motion/data frames)
typedef union {
  uint8_t buffer[XFER_BUF_SIZE];
  struct {
    int32_t header;
    float stepgen_position_cmd[STEPGENS];  // machine units
    int32_t output_vars[OUTPUT_VARS];
    uint8_t stepgen_enable_mask;
    uint16_t outputs;
  };
} linuxCncState_t;

// struct for LinuxCNC -> PRU configuration (one-shot on link/reset)
typedef union {
  uint8_t buffer[XFER_BUF_SIZE];
  struct {
    int32_t header;
    float stepper_init_position[STEPGENS];     // machine units
    float stepper_max_accel[STEPGENS];         // mu/s^2
    float stepper_position_scale[STEPGENS];    // steps per mu
    float servo_period_s;                      // host servo period (seconds)
  };
} linuxCncConf_t;

// struct for PRU -> LinuxCNC communication
typedef union {
  uint8_t buffer[XFER_BUF_SIZE];
  struct {
    int32_t header;
    int64_t stepgen_feedback[STEPGENS];
    int32_t input_vars[INPUT_VARS];
    uint16_t inputs;
  };
} pruState_t;

#endif  // PROTOCOL_DEFINITIONS_H
