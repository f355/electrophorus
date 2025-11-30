#pragma once

#include "mbed.h"
#include "modules/module.h"
#include "pin.h"

class ADC final : public Module {
  uint8_t var_number_;
  AnalogIn* adc_;

  uint16_t value_;
  uint32_t run_every_;
  uint32_t counter_;

 public:
  ADC(uint8_t var_number, Pin* pin);

  void OnRx() override;
};
