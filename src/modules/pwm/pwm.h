#ifndef PWM_H
#define PWM_H

#include "module.h"
#include "pin.h"
#include "spi_comms.h"

class PWM final : public Module {
  PwmOut *pwm_pin;  // PWM out object

  SpiComms* comms;
  int var_number;

  int period_us;       // Period (us)
  int32_t duty_cycle;  // Pulse width (tenths of %, per mil)

 public:
  PWM(int var_number, const Pin *pin, int period_us, SpiComms* comms);

  bool listens_to_rx() override;
  void on_rx() override;
};

#endif
