#ifndef MACHINE_DEFINITIONS_H
#define MACHINE_DEFINITIONS_H

#include "machinedef_types.h"

#define BASE_FREQUENCY 100000  // 100 kHz
#define SERVO_FREQUENCY 1000   // 1 kHz

#define STEPPERS 4

#define STEPPER_NAMES {"x", "y", "z", "a"};

#define INPUT_PINS 14

// clang-format off
#define INPUT_PIN_DESC \
  {{"e-stop", 0, 20, false, false}, \
   {"stall-alarm-spindle", 0, 19, false, false}, \
   {"stall-alarm-x", 0, 1, false, false}, \
   {"stall-alarm-y", 0, 0, false, false}, \
   {"stall-alarm-z", 3, 25, false, false}, \
   {"lid-sensor", 1, 8, false, true}, \
   {"main-button", 2, 13, false, true}, \
   {"ext-port", 0, 21, true, false}, \
   {"endstop-x", 0, 24, false, false}, \
   {"endstop-y", 0, 25, false, false}, \
   {"endstop-z", 1, 1, false, false}, \
   {"endstop-a", 1, 9, false, false}, \
   {"probe", 2, 6, true, false}, \
   {"tool-setter", 0, 5, false, false}}
// clang-format on

#define OUTPUT_PINS 6
// clang-format off
#define OUTPUT_PIN_DESC \
  {{"work-light", 2, 0, false}, \
   {"probe-power", 0, 11, false}, \
   {"beeper", 1, 14, false}, \
   {"power-12v", 0, 22, false}, \
   {"power-24v", 0, 10, false}, \
   {"axis-enable-a", 1, 30, true}}
// clang-format on

#define INPUT_VARS 3
#define INPUT_VAR_NAMES {"spindle-feedback", "spindle-temperature", "power-supply-temperature"}

#define OUTPUT_VARS 4
#define OUTPUT_VAR_NAMES {"spindle-duty", "spindle-fan-duty", "power-supply-fan-duty", "ext-port-duty"}

#endif
