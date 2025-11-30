#include "pwm.h"

#include "mbed.h"
#include "pin.h"
#include "spi_comms.h"

PWM::PWM(const uint8_t var_number, const uint8_t period_var_number, const Pin* pin)
    : var_number_(var_number),
      period_var_number_(period_var_number),
      pwm_pin_(new PwmOut(pin->ToPinName())),
      duty_cycle_(0) {}

void PWM::OnRx() {
  const auto vars = SpiComms::Instance()->get_linuxcnc_state()->output_vars;

  // on LPC1768, the period is shared among all PWMs,
  // so we shouldn't try setting it to different values - the last one wins - hence the static declaration.
  // many bothans died to bring us this information.
  static int32_t period_us = 0;
  if (const auto new_period = vars[period_var_number_]; new_period != 0 && new_period != period_us) {
    period_us = new_period;
    pwm_pin_->period_us(period_us);
  }

  if (const auto new_duty_cycle = vars[var_number_]; new_duty_cycle != duty_cycle_) {
    duty_cycle_ = new_duty_cycle;
    pwm_pin_->pulsewidth_us(duty_cycle_ * period_us / 1000);
  }
}
