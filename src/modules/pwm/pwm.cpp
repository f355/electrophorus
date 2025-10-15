#include "pwm.h"

PWM::PWM(const uint8_t var_number, const Pin* pin, SpiComms* comms)
    : comms(comms), var_number(var_number), pwm_pin(new PwmOut(pin->to_pin_name())), duty_cycle(0) {
  this->pwm_pin->period_us(PWM_PERIOD_US);
}

void PWM::on_rx() {
  const int32_t new_duty_cycle = this->comms->get_linuxcnc_state()->output_vars[this->var_number];
  if (this->duty_cycle != new_duty_cycle) {
    this->duty_cycle = new_duty_cycle;
    this->pwm_pin->pulsewidth_us(this->duty_cycle * PWM_PERIOD_US / 1000);
  }
}
