#include "pwm.h"

PWM::PWM(const uint8_t var_number, const Pin* pin)
    : var_number_(var_number), pwm_pin_(new PwmOut(pin->ToPinName())), duty_cycle_(0) {
  pwm_pin_->period_us(kPwmPeriodUs);
}

void PWM::OnRx() {
  const int32_t new_duty_cycle = SpiComms::Instance()->get_linuxcnc_state()->output_vars[var_number_];
  if (duty_cycle_ != new_duty_cycle) {
    duty_cycle_ = new_duty_cycle;
    pwm_pin_->pulsewidth_us(duty_cycle_ * kPwmPeriodUs / 1000);
  }
}
