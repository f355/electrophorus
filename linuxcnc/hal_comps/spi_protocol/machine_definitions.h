#pragma once

#include "machinedef_types.h"

// Proxxon PD 250/E lathe using Carvera C1 board

constexpr auto kStepgenTickFrequency = 100'000;  // 100 kHz

constexpr auto kNumStepgens = 2;
constexpr const char* kStepgenNames[kNumStepgens] = {"x", "z"};

constexpr auto kNumInputPins = 4;
constexpr inputPin_t kInputPins[kNumInputPins] = {
    {"e-stop", 0, 26, false, true},
    {"home-x", 0, 24, false, false},
    {"home-z", 0, 25, false, false},
    {"tool-setter", 0, 5, false, false},
};

constexpr auto kNumOutputPins = 3;
constexpr outputPin_t kOutputPins[kNumOutputPins] = {
    {"spindle-enable", 2, 0, false},  // WS55-220 EN
    {"spindle-dir", 0, 23, false},    // WS55-220 FR
    {"power-24v", 0, 10, false},
};

constexpr auto kNumInputVars = 0;
constexpr const char* kInputVarNames[kNumInputVars] = {};

constexpr auto kNumOutputVars = 2;
constexpr const char* kOutputVarNames[kNumOutputVars] = {"pwm-period-us", "spindle-duty"};
