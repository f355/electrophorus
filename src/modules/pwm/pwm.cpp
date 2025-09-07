#include "pwm.h"

#include <algorithm>

PWM::PWM(const int var_number, const Pin* pin, const int period_us, const SerialComms* comms)
    : comms(comms), var_number(var_number), pwm_pin(nullptr), period_us(period_us), duty_cycle(0) {
#ifndef EPHO_NO_HW_IO
  pwm_pin = new PwmOut(pin->to_pin_name());
  pwm_pin->period_us(period_us);
#else
  (void)pin;  // no hardware in debug mode
#endif
}

void PWM::on_rx() {
  const int32_t raw = this->comms->get_linuxcnc_state()->output_vars[var_number];
  if (const int32_t clamped = std::clamp(raw, 0L, 1000L); this->duty_cycle != clamped) {
    this->duty_cycle = clamped;
#ifndef EPHO_NO_HW_IO
    if (this->pwm_pin) {
      this->pwm_pin->pulsewidth_us(this->period_us * this->duty_cycle / 1000);
    }
#endif
  }
}

bool PWM::listens_to_rx() { return true; }
