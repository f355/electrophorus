#include "stepgen.h"

Stepgen::Stepgen(const int stepper_number, Pin* step_pin, Pin* dir_pin, const uint32_t ticker_frequency,
                 volatile rxData_t* rx_data, volatile txData_t* tx_data)
    : stepper_enable_mask(1 << stepper_number),
      stepper_number(stepper_number),
      step_position(&tx_data->stepgen_feedback[stepper_number]),
      stepper_enable(&rx_data->stepgen_enable_mask),
      rx_data(rx_data),
      ticker_frequency(ticker_frequency),
      step_pin(step_pin->as_output()),
      dir_pin(dir_pin->as_output()) {
  this->dir_pin->set(this->current_dir);
}

void Stepgen::run_base() {
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

  if ((*this->stepper_enable & this->stepper_enable_mask) == 0) {
    return;  // stepper is disabled, nothing to do
  }

  // Direct Digital Synthesis (DDS)
  // works by incrementing a fixed-point accumulator on every tick with a value calculated such that the accumulator
  // reaches fixed-point 1.0 at the commanded frequency.
  //
  // That value is, unsurprisingly, the fixed-point commanded frequency divided by the integer ticker frequency - to get
  // steps/tick, you divide steps/second by ticks/second.

  if (this->increment == 0) return;

  int64_t position = *this->step_position;
  const int64_t old_position = position;
  position += increment;

  if ((old_position ^ position) & FIXED_ONE) {
    // the next whole step value is reached, make a step
    this->step_pin->set(true);
    this->is_stepping = true;
  }
  *this->step_position = position;
}

void Stepgen::on_rx() {
  // Handle config payload
  if (this->rx_data->header == PRU_CONFIG) {
    float maxvel = this->rx_data->stepgen_maxvel[this->stepper_number];
    float maxaccel = this->rx_data->stepgen_maxaccel[this->stepper_number];
    float init_pos_steps = this->rx_data->stepgen_init_pos[this->stepper_number];

    this->maxvel = maxvel;
    this->maxaccel = maxaccel;

    // Initialize DDS position from LinuxCNC
    *this->step_position = static_cast<int64_t>(init_pos_steps * FIXED_ONE);
    this->old_position_cmd = init_pos_steps;

    this->config_received = true;

    printf("Stepgen %d: config received - maxvel=%.1f, maxaccel=%.1f, init_pos=%.1f\n",
           this->stepper_number, this->maxvel, this->maxaccel, init_pos_steps);
  }
}

void Stepgen::run_servo() {
  if (!this->config_received || (*this->stepper_enable & this->stepper_enable_mask) == 0) {
    return;
  }

  // Get current position feedback from step accumulator (in steps)
  float position_fb = static_cast<float>(*this->step_position) / FIXED_ONE;

  // Calculate feed-forward velocity
  float current_position_cmd = this->rx_data->stepgen_position_cmd[this->stepper_number];
  float ff_vel = (current_position_cmd - this->old_position_cmd) / SERVO_PERIOD;
  this->old_position_cmd = current_position_cmd;

  // Calculate velocity error (current velocity vs desired feed-forward)
  float velocity_error = this->velocity_fb - ff_vel;

  // Determine required acceleration to match velocity
  float match_accel;
  if (velocity_error > 0.0f) {
    if (this->maxaccel == 0) {
      match_accel = -velocity_error / SERVO_PERIOD;
    } else {
      match_accel = -this->maxaccel;
    }
  } else if (velocity_error < 0.0f) {
    if (this->maxaccel == 0) {
      match_accel = -velocity_error / SERVO_PERIOD;
    } else {
      match_accel = this->maxaccel;
    }
  } else {
    match_accel = 0;
  }

  // Calculate time to velocity match
  float seconds_to_vel_match;
  if (match_accel == 0) {
    seconds_to_vel_match = 0.0f;
  } else {
    seconds_to_vel_match = -velocity_error / match_accel;
  }

  // Predict position at velocity match
  float avg_v = (ff_vel + this->velocity_fb) * 0.5f;
  float position_at_match = position_fb + avg_v * (seconds_to_vel_match + SERVO_PERIOD);

  // Predict command position at velocity match
  float position_cmd_at_match = current_position_cmd + ff_vel * seconds_to_vel_match;
  float error_at_match = position_at_match - position_cmd_at_match;

  float velocity_cmd;
  if (seconds_to_vel_match < SERVO_PERIOD) {
    // Can match velocity in one period - apply position error correction
    velocity_cmd = ff_vel - (0.5f * error_at_match / SERVO_PERIOD);

    // Apply acceleration limits
    if (this->maxaccel > 0) {
      float max_vel_change = this->maxaccel * SERVO_PERIOD;
      if (velocity_cmd > this->velocity_fb + max_vel_change) {
        velocity_cmd = this->velocity_fb + max_vel_change;
      } else if (velocity_cmd < this->velocity_fb - max_vel_change) {
        velocity_cmd = this->velocity_fb - max_vel_change;
      }
    }
  } else {
    // Multi-period velocity matching with predictive correction
    float dv = -2.0f * match_accel * SERVO_PERIOD;
    float dp = dv * seconds_to_vel_match;

    // Choose acceleration direction to minimize future error
    if (fabsf(error_at_match + dp * 2.0f) < fabsf(error_at_match)) {
      match_accel = -match_accel;
    }

    velocity_cmd = this->velocity_fb + match_accel * SERVO_PERIOD;
  }

  // Apply velocity limits
  if (this->maxvel > 0) {
    if (velocity_cmd > this->maxvel) velocity_cmd = this->maxvel;
    else if (velocity_cmd < -this->maxvel) velocity_cmd = -this->maxvel;
  }

  // Update velocity feedback for next iteration
  this->velocity_fb = velocity_cmd;

  // Convert to DDS increment (velocity_cmd is already in steps/sec)
  this->increment = static_cast<int64_t>(velocity_cmd * (static_cast<float>(FIXED_ONE) / this->ticker_frequency));

  // Handle direction changes
  if (const bool is_forward = increment > 0; this->current_dir != is_forward) {
    this->current_dir = is_forward;
    this->dir_flipped = true;
    this->dir_pin->set(is_forward);
  }
}

bool Stepgen::listens_to_rx() { return true; }

bool Stepgen::is_base() { return true; }

bool Stepgen::is_servo() { return true; }
