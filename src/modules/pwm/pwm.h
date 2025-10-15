#ifndef PWM_H
#define PWM_H

#include "modules/module.h"
#include "pin.h"
#include "spi_comms.h"

class PWM final : public Module {
  SpiComms *comms;
  uint8_t var_number;

  PwmOut *pwm_pin;  // PWM out object

  int32_t duty_cycle;  // Pulse width (tenths of %, per mil)

 public:
  PWM(uint8_t var_number, const Pin *pin, SpiComms *comms);

  void on_rx() override;
};

#endif
