#include "stepgen.h"

Stepgen::Stepgen(const int stepper_number, Pin* step_pin, Pin* dir_pin, const uint32_t ticker_frequency,
                 SerialComms* comms)
    : comms(comms),
      stepper_number(stepper_number),
      stepper_enable_mask(1 << stepper_number),
      ticker_frequency(ticker_frequency),
      step_position(0),
      step_pin(step_pin->as_output()),
      dir_pin(dir_pin->as_output()) {
  this->dir_pin->set(this->current_dir);
}

void Stepgen::make_steps() {
  if (this->is_stepping) {
    // bring down the step pin that was set high on the previous tick
    this->step_pin->set(false);
    this->is_stepping = false;
  }

  if (this->dir_flipped) {
    // the direction of travel has changed, do not step this iteration to give some setup time.
    // TODO(f355): make hold time configurable.
    this->dir_flipped = false;
    return;
  }

  if ((this->comms->get_linuxcnc_state()->stepgen_enable_mask & this->stepper_enable_mask) == 0) {
    return;  // stepper is disabled, nothing to do
  }

  // Direct Digital Synthesis (DDS)
  // works by incrementing a fixed-point accumulator on every tick with a value calculated such that the accumulator
  // reaches fixed-point 1.0 at the commanded frequency.
  //
  // That value is, unsurprisingly, the fixed-point commanded frequency divided by the integer ticker frequency - to get
  // steps/tick, you divide steps/second by ticks/second.

  if (this->increment == 0) return;

  const int64_t old_position = this->step_position;
  this->step_position += increment;

  if ((old_position ^ this->step_position) & FIXED_ONE) {
    // the next whole step value is reached, make a step
    this->step_pin->set(true);
    this->is_stepping = true;
  }
  this->comms->get_pru_state()->stepgen_feedback[stepper_number] = this->step_position;
}

void Stepgen::on_rx() {
  // Apply pending configuration first, even if the axis is disabled
  float sc = 0.0f, ac = 0.0f, ip = 0.0f;
  if (this->comms->take_stepgen_conf(this->stepper_number, &sc, &ac, &ip)) {
    this->position_scale = sc;
    this->max_accel = ac;
    this->step_position = static_cast<int64_t>((double)ip * (double)sc * (double)FIXED_ONE);
    // publish updated position immediately so host sees correct initial position
    this->comms->get_pru_state()->stepgen_feedback[stepper_number] = this->step_position;
  }

  const auto* l = this->comms->get_linuxcnc_state();
  if ((l->stepgen_enable_mask & this->stepper_enable_mask) == 0) return;  // disabled

  const float pos_mu = l->stepgen_position_cmd[this->stepper_number];

  // feedforward velocity in mu/s from consecutive host samples
  const float T = this->comms->get_servo_period_s() > 0.0f ? this->comms->get_servo_period_s() : 0.00125f;
  const float ff_vel_mu_s = (pos_mu - this->last_position_cmd_mu) / T;

  // accel-limit toward ff_vel
  float target_vel = ff_vel_mu_s;
  if (this->max_accel > 0.0f) {
    const float dv_max = this->max_accel * T;
    const float dv = target_vel - this->last_velocity_mu_s;
    if (dv > dv_max) target_vel = this->last_velocity_mu_s + dv_max;
    else if (dv < -dv_max) target_vel = this->last_velocity_mu_s - dv_max;
  }

  // convert to steps/s and then to fixed-point increment per stepgen tick
  const float steps_per_s = target_vel * this->position_scale;
  this->increment = static_cast<int64_t>(steps_per_s * (static_cast<float>(FIXED_ONE) / this->ticker_frequency));

  // Update direction on change
  if (const bool is_forward = increment > 0; this->current_dir != is_forward) {
    this->current_dir = is_forward;
    this->dir_flipped = true;
    this->dir_pin->set(is_forward);
  }

  this->last_position_cmd_mu = pos_mu;
  this->last_velocity_mu_s = target_vel;
}

bool Stepgen::listens_to_rx() { return true; }

bool Stepgen::is_stepgen() { return true; }
