#include "stepgen.h"

#include <cmath>

Stepgen::Stepgen(const int stepper_number, Pin* step_pin, Pin* dir_pin, volatile rxData_t* rx_data,
                 volatile txData_t* tx_data)
    : stepper_number(stepper_number),
      stepper_enable_mask(1 << stepper_number),
      max_accel(100.0f),
      position_scale(1.0f),
      rx_data(rx_data),
      tx_data(tx_data),
      old_position_command(0.0f),
      current_velocity(0.0f),
      step_position(0),
      increment(0),
      step_pin(step_pin->as_output()),
      dir_pin(dir_pin->as_output()) {
  this->dir_pin->set(this->current_dir);
}

void Stepgen::run_base() {
  if (!configured) return;

  if (this->is_stepping) {
    // bring down the step pin that was set high on the previous tick
    this->step_pin->set(false);
    this->is_stepping = false;
  }

  if (this->dir_flipped) {
    // if the direction has just flipped, skip a step to give the stepper driver some setup time
    this->dir_flipped = false;
    return;
  }

  if ((this->rx_data->stepgen_enable_mask & this->stepper_enable_mask) == 0) {
    return;  // stepper is disabled, nothing to do
  }

  const int64_t inc = this->increment;

  if (inc == 0) return;

  const int64_t old_position = this->step_position;
  this->step_position += inc;

  // Convert DDS position to machine units for feedback
  const float current_position_mu = static_cast<float>(this->step_position) / (this->position_scale * DDS_SCALE);
  this->tx_data->stepgen_feedback[this->stepper_number] = current_position_mu;

  // Check if we crossed a step boundary (bit 32 flipped)
  if ((this->step_position ^ old_position) & DDS_SCALE) {
    // the next whole step value is reached, make a step
    this->step_pin->set(true);
    this->is_stepping = true;
  }
}
void Stepgen::run_servo() {
  switch (rx_data->header) {
    case PRU_CONF: {
      const uint8_t s = this->stepper_number;
      this->position_scale = rx_data->stepper_position_scale[s];
      this->max_accel = rx_data->stepper_max_accel[s] * this->position_scale;
      if (this->max_accel == 0.0f) {
        // TODO(f355): instead of using a rather arbitrary value, provide error feedback to LinuxCNC
        this->max_accel = this->position_scale * 100.0f;
      }
      const float init_pos = rx_data->stepper_init_position[s];
      this->old_position_command = init_pos * this->position_scale;
      this->step_position = static_cast<int64_t>(this->old_position_command * DDS_SCALE);
      this->tx_data->stepgen_feedback[s] = init_pos;
      this->configured = true;
      break;
    }
    default:
      this->calc_velocity();
  }
}

void Stepgen::calc_velocity() {
  if (!configured) return;

  constexpr float dt = 1.0f / SERVO_FREQUENCY;

  // Commanded position (external input, scaled to steps)
  const float commanded_pos = this->rx_data->stepgen_freq_command[stepper_number] * this->position_scale;

  // Feed-forward velocity (steps/s)
  const float vel_ff = (commanded_pos - this->old_position_command) / dt;
  this->old_position_command = commanded_pos;

  const float vel_err = this->current_velocity - vel_ff;
  const float time_to_match = -vel_err / this->max_accel;

  // Feedback position projected at velocity match
  const float vel_avg = (vel_ff + this->current_velocity) / 2.0f;
  const float current_pos_steps = static_cast<float>(this->step_position) / (this->position_scale * DDS_SCALE);
  const float pos_at_match = current_pos_steps + vel_avg * (time_to_match + dt);

  // Commanded position projected at velocity match
  const float cmd_pos_at_match = commanded_pos + vel_ff * time_to_match;

  // Position error at velocity match
  const float err_at_match = pos_at_match - cmd_pos_at_match;

  // --- Velocity planning ---
  float proposed_vel;

  if (time_to_match < dt + 0.000001f) {  // Small buffer for numerical stability
    // velocity can match in one tick → correct position error
    proposed_vel = vel_ff - (err_at_match / (dt * 2.0f));
  } else {
    // velocity match takes >1 tick
    const float time_factor = time_to_match + dt;

    // Base error if we held the current velocity:
    const float base_err = current_pos_steps - cmd_pos_at_match + this->current_velocity * time_factor;

    // Offset contribution from accel choice
    const float offset = (this->max_accel * dt / 2.0f) * time_factor;

    // Pick a direction that minimizes |error|
    const float accel = (fabsf(base_err + offset) < fabsf(base_err - offset)) ? this->max_accel : -this->max_accel;

    proposed_vel = this->current_velocity + accel * dt;
  }

  // --- Apply accel limits ---
  const float max_step = this->max_accel * dt;
  proposed_vel = std::clamp(proposed_vel, this->current_velocity - max_step, this->current_velocity + max_step);

  // --- DDS update if velocity changed ---
  if (proposed_vel != this->current_velocity) {
    this->current_velocity = proposed_vel;

    // Convert velocity to DDS increment (steps/s to DDS units)
    const auto inc = static_cast<int64_t>(proposed_vel * DDS_SCALE / BASE_FREQUENCY);
    this->increment = inc;

    if (this->current_dir != (inc > 0)) {
      this->current_dir = !this->current_dir;
      this->dir_flipped = true;
      this->dir_pin->set(this->current_dir);
    }
  }
}