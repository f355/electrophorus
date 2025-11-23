#pragma once

#include "machinedef_types.h"

constexpr auto kStepgenTickFrequency = 100'000;  // 100 kHz

constexpr auto kNumStepgens = 4;
constexpr const char* kStepgenNames[kNumStepgens] = {"x", "y", "z", "a"};

constexpr auto kNumInputPins = 14;
constexpr inputPin_t kInputPins[kNumInputPins] = {
    {"e-stop", 0, 20, false, true},         {"stall-alarm-spindle", 0, 19, false, false},
    {"stall-alarm-x", 0, 1, false, false},  {"stall-alarm-y", 0, 0, false, false},
    {"stall-alarm-z", 3, 25, false, false}, {"lid-sensor", 1, 8, false, true},
    {"main-button", 2, 13, false, true},    {"ext-port", 0, 21, true, false},
    {"endstop-x", 0, 24, false, false},     {"endstop-y", 0, 25, false, false},
    {"endstop-z", 1, 1, false, false},      {"endstop-a", 1, 9, false, false},
    {"probe", 2, 6, true, false},           {"tool-setter", 0, 5, false, false}};

constexpr auto kNumOutputPins = 6;
constexpr outputPin_t kOutputPins[kNumOutputPins] = {{"work-light", 2, 0, false}, {"probe-power", 0, 11, false},
                                                     {"beeper", 1, 14, false},    {"power-12v", 0, 22, false},
                                                     {"power-24v", 0, 10, false}, {"axis-enable-a", 1, 30, true}};

constexpr auto kNumInputVars = 3;
constexpr const char* kInputVarNames[kNumInputVars] = {"spindle-feedback", "spindle-temperature",
                                                       "power-supply-temperature"};
constexpr auto kNumOutputVars = 5;
constexpr const char* kOutputVarNames[kNumOutputVars] = {"pwm-period-us", "spindle-duty", "spindle-fan-duty",
                                                         "power-supply-fan-duty", "ext-port-duty"};
