#pragma once

#include "modules/module.h"
#include "pin.h"
#include "spi_comms.h"

class PWM final : public Module {
  uint8_t var_number_;

  PwmOut *pwm_pin_;  // PWM out object

  int32_t duty_cycle_;  // Pulse width (tenths of %, per mil)

 public:
  PWM(uint8_t var_number, const Pin *pin);

  void OnRx() override;
};
