#ifndef PWM_H
#define PWM_H

#include "comms.h"
#include "module.h"
#include "pin.h"

class PWM final : public Module {
  PwmOut *pwm_pin;  // PWM out object

  volatile int32_t *set_duty_cycle;

  int period_us;       // Period (us)
  int32_t duty_cycle;  // Pulse width (tenths of %, per mil)
  int pulse_width_us;  // Pulse width (us)

 public:
  PWM(int var_number, const Pin *pin, int period_us, volatile rxData_t *rx_data);

  void run() override;
};

#endif
