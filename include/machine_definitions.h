#ifndef MACHINE_DEFINITIONS_H
#define MACHINE_DEFINITIONS_H

#define SPI_MOSI P0_18
#define SPI_MISO P0_17
#define SPI_SCK P0_15
#define SPI_SSEL P0_16

#define BASE_THREAD_FREQUENCY 100000  // 100 kHz
#define SERVO_THREAD_FREQUENCY 1000   // 1 kHz

#define JOINTS 4

typedef struct {
  const char *name;
  uint8_t port;
  uint8_t number;
  bool pull_down;
  bool invert;
} inputPin_t;

#define INPUT_PINS 13
#define INPUT_PIN_DESC                                                                                             \
  {                                                                                                                \
    {"stall-alarm-spindle", 0, 19, false, false}, {"stall-alarm-x", 0, 1, false, false},                           \
        {"stall-alarm-y", 0, 0, false, false}, {"stall-alarm-z", 3, 25, false, false},                             \
        {"lid-sensor", 1, 8, false, true}, {"main-button", 2, 13, false, true}, {"ext-port", 0, 21, true, false},  \
        {"endstop-x", 0, 24, false, false}, {"endstop-y", 0, 25, false, false}, {"endstop-z", 1, 1, false, false}, \
        {"endstop-a", 1, 9, false, false}, {"probe", 2, 6, true, false}, {                                         \
      "tool-length-sensor", 0, 5, false, false                                                                     \
    }                                                                                                              \
  }

typedef struct {
  const char *name;
  uint8_t port;
  uint8_t number;
  bool invert;
} outputPin_t;

#define OUTPUT_PINS 6
#define OUTPUT_PIN_DESC                                                                                                \
  {                                                                                                                    \
    {"work-light", 2, 0, false}, {"probe-power", 0, 11, false}, {"beeper", 1, 14, false}, {"power-12v", 0, 22, false}, \
        {"power-24v", 0, 10, false}, {                                                                                 \
      "axis-enable-a", 1, 30, true                                                                                     \
    }                                                                                                                  \
  }

#define INPUT_VARS 3
#define INPUT_VAR_NAMES {"spindle-feedback", "spindle-temperature", "power-supply-temperature"}

#define OUTPUT_VARS 4
#define OUTPUT_VAR_NAMES {"spindle-duty", "spindle-fan-duty", "power-supply-fan-duty", "ext-port-duty"}

#endif
