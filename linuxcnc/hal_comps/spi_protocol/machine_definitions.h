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

constexpr auto kNumOutputPins = 6;
constexpr outputPin_t kOutputPins[kNumOutputPins] = {{"spindle-enable", 2, 0, false}, {"spindle-dir", 0, 23, false},
                                                     {"power-12v", 0, 22, false},     {"power-24v", 0, 10, false},
                                                     {"x-enable", 3, 26, true},       {"z-enable", 1, 30, true}};

constexpr auto kNumInputVars = 2;
constexpr const char* kInputVarNames[kNumInputVars] = {"spindle-pos-fb", "spindle-vel-fb"};

constexpr auto kNumOutputVars = 2;
constexpr const char* kOutputVarNames[kNumOutputVars] = {"pwm-period-us", "spindle-duty"};
