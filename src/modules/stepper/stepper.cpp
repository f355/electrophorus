#include "stepper.h"

Stepper::Stepper(const int stepper_number, Pin* step_pin, Pin* dir_pin, volatile rxData_t* rx_data,
                 volatile txData_t* tx_data)
    : stepper_number(stepper_number),
      stepper_enable_mask(1 << stepper_number),
      rx_data(rx_data),
      tx_data(tx_data),
      step_pin(step_pin->as_output()),
      dir_pin(dir_pin->as_output()) {
  this->dir_pin->set(this->current_dir);
}

void Stepper::run_base() {
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

  if ((this->rx_data->stepper_enable_mask & this->stepper_enable_mask) == 0) {
    return;  // stepper is disabled, nothing to do
  }

  const Fixed32_32 inc = this->increment;

  if (inc == 0) return;

  Fixed32_32 current_position = this->step_position;
  const Fixed32_32 old_position = current_position;
  current_position += inc;
  this->step_position.assign_from(current_position);
  this->tx_data->stepper_position_fb[this->stepper_number] = (current_position / this->position_scale).raw_value();

  if (current_position.one_over(old_position)) {
    // the next whole step value is reached, make a step
    this->step_pin->set(true);
    this->is_stepping = true;
  }
}
void Stepper::run_servo() {
  switch (rx_data->header) {
    case PRU_CONF: {
      uint8_t s = this->stepper_number;
      this->position_scale = Fixed32_32::from_raw(static_cast<int64_t>(rx_data->stepper_position_scale[s]) << 24);
      this->max_accel =
          Fixed32_32::from_raw(static_cast<int64_t>(rx_data->stepper_max_accel[s]) << 24) * this->position_scale;
      if (this->max_accel == 0) {
        // TODO(f355): instead of using a rather arbitrary value, provide error feedback to LinuxCNC
        this->max_accel = position_scale * 100;
      }
      const Fixed32_32 init_pos = Fixed32_32::from_raw(static_cast<int64_t>(rx_data->stepper_init_position[s]) << 24);
      this->step_position.assign_from(init_pos * this->position_scale);
      this->tx_data->stepper_position_fb[s] = init_pos.raw_value();
      this->configured = true;
      break;
    }
    default:
      this->calc_velocity();
  }
}

void Stepper::calc_velocity() {
  if (!configured) return;

  constexpr Fixed32_32 dt = 1 / SERVO_FREQUENCY;

  // Commanded position (external input, scaled to steps)
  const Fixed32_32 commanded_pos =
      Fixed32_32::from_raw(this->rx_data->stepper_pos_command[stepper_number]) * this->position_scale;

  // Feed-forward velocity (steps/s)
  const Fixed32_32 vel_ff = (commanded_pos - this->old_position_command) / dt;
  this->old_position_command = commanded_pos;

  const Fixed32_32 vel_err = this->current_velocity - vel_ff;
  const Fixed32_32 time_to_match = -vel_err / this->max_accel;

  // Feedback position projected at velocity match
  const Fixed32_32 vel_avg = (vel_ff + this->current_velocity) / 2;
  const Fixed32_32 pos_at_match = this->step_position + vel_avg * (time_to_match + dt);

  // Commanded position projected at velocity match
  const Fixed32_32 cmd_pos_at_match = commanded_pos + vel_ff * time_to_match;

  // Position error at velocity match
  const Fixed32_32 err_at_match = pos_at_match - cmd_pos_at_match;

  // --- Velocity planning ---
  Fixed32_32 proposed_vel;

  if (time_to_match < dt + Fixed32_32(1) / 1000000) {  // Small buffer for numerical stability
    // velocity can match in one tick → correct position error
    proposed_vel = vel_ff - (err_at_match / (dt * 2));
  } else {
    // velocity match takes >1 tick
    const Fixed32_32 time_factor = time_to_match + dt;

    // Base error if we held the current velocity:
    const Fixed32_32 base_err = this->step_position - cmd_pos_at_match + this->current_velocity * time_factor;

    // Offset contribution from accel choice
    const Fixed32_32 offset = (this->max_accel * dt / 2) * time_factor;

    // Pick a direction that minimizes |error|
    const Fixed32_32 accel =
        ((base_err + offset).abs() < (base_err - offset).abs()) ? this->max_accel : -this->max_accel;

    proposed_vel = this->current_velocity + accel * dt;
  }

  // --- Apply accel limits ---
  const Fixed32_32 max_step = this->max_accel * dt;
  proposed_vel = std::clamp(proposed_vel, this->current_velocity - max_step, this->current_velocity + max_step);

  // --- DDS update if velocity changed ---
  if (proposed_vel != this->current_velocity) {
    this->current_velocity = proposed_vel;

    const Fixed32_32 inc = proposed_vel / BASE_FREQUENCY;
    this->increment.assign_from(inc);

    if (this->current_dir != (inc > 0)) {
      this->current_dir = !this->current_dir;
      this->dir_flipped = true;
      this->dir_pin->set(this->current_dir);
    }
  }
}