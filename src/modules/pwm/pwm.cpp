#include "pwm.h"

PWM::PWM(const int var_number, const Pin* pin, const int period_us, SpiComms* comms)
    : pwm_pin(new PwmOut(pin->to_pin_name())),
      comms(comms),
      var_number(var_number),
      period_us(period_us),
      duty_cycle(0) {}

void PWM::on_rx() {
  const int32_t new_dc = this->comms->get_linuxcnc_state()->output_vars[this->var_number];
  if (this->duty_cycle != new_dc) {
    this->duty_cycle = new_dc;
    this->pwm_pin->pulsewidth_us(this->period_us * this->duty_cycle / 1000);
  }
}

bool PWM::listens_to_rx() { return true; }
