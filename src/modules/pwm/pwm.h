#ifndef PWM_H
#define PWM_H

#include "module.h"
#include "pin.h"
#include "serial_comms.h"

class PWM final : public Module {
  const SerialComms *comms;
  int var_number;

  PwmOut *pwm_pin;

  int period_us;       // Period (us)
  int32_t duty_cycle;  // Pulse width (tenths of %, per mil)

 public:
  PWM(int var_number, const Pin *pin, int period_us, const SerialComms *comms);

  bool listens_to_rx() override;
  void on_rx() override;
};

#endif
