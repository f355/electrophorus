#pragma once

#include "modules/module.h"
#include "pin.h"
#include "spi_comms.h"

class PulseCounter final : public Module {
  uint8_t var_number_;

  volatile int32_t counter_ = 0;
  void InterruptHandler();

 public:
  PulseCounter(uint8_t var_number, const Pin* pin);

  void OnRx() override;
};
