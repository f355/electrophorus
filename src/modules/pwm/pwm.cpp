#include "pwm.h"

PWM::PWM(const int var_number, const Pin* pin, const int period_us, volatile rxData_t* rx_data)
    : pwm_pin(new PwmOut(pin->to_pin_name())),
      set_duty_cycle(&rx_data->output_vars[var_number]),
      period_us(period_us),
      duty_cycle(0) {}

void PWM::run() {
  if (this->duty_cycle != *this->set_duty_cycle) {
    this->duty_cycle = *this->set_duty_cycle;
    this->pwm_pin->pulsewidth_us(this->period_us * this->duty_cycle / 1000);
  }
}
