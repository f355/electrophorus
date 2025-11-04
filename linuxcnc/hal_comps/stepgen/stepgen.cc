#include "stepgen.h"

#include <algorithm>
#include <cmath>
#include <string>

int Stepgen::Init(const int comp_id, const char *const modname, const char *const prefix, const int i) {
  this->index_ = i;
  param_ = static_cast<Params *>(hal_malloc(sizeof(Params)));
  if (!param_) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed for stepgen params\n", modname);
    return -1;
  }
  pin_ = static_cast<Pins *>(hal_malloc(sizeof(Pins)));
  if (!pin_) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed for stepgen pin pointers\n", modname);
    return -1;
  }

  const std::string base = std::string(prefix) + ".stepgen." + kStepgenNames[i];
  const char *b = base.c_str();

  // Params (storage must live in HAL shared memory)
  if (hal_param_float_newf(HAL_RW, &param_->position_scale, comp_id, "%s.position-scale", b) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: stepgen param export failed (position-scale)\n", modname);
    return -1;
  }
  if (hal_param_float_newf(HAL_RW, &param_->maxvel, comp_id, "%s.maxvel", b) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: stepgen param export failed (maxvel)\n", modname);
    return -1;
  }
  if (hal_param_float_newf(HAL_RW, &param_->maxaccel, comp_id, "%s.maxaccel", b) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: stepgen param export failed (maxaccel)\n", modname);
    return -1;
  }

  param_->position_scale = 1.0;
  param_->maxvel = 0.0;
  param_->maxaccel = 1.0;

  // Pins - pointer variables themselves must be in HAL shared memory
  if (hal_pin_bit_newf(HAL_IN, &pin_->enable, comp_id, "%s.enable", b) < 0) return -1;
  if (hal_pin_bit_newf(HAL_IN, &pin_->control_type, comp_id, "%s.control-type", b) < 0) return -1;
  if (hal_pin_bit_newf(HAL_IN, &pin_->position_reset, comp_id, "%s.position-reset", b) < 0) return -1;
  if (hal_pin_float_newf(HAL_IN, &pin_->position_cmd, comp_id, "%s.position-cmd", b) < 0) return -1;
  if (hal_pin_float_newf(HAL_IN, &pin_->velocity_cmd, comp_id, "%s.velocity-cmd", b) < 0) return -1;
  if (hal_pin_float_newf(HAL_OUT, &pin_->position_fb, comp_id, "%s.position-fb", b) < 0) return -1;
  if (hal_pin_float_newf(HAL_OUT, &pin_->velocity_fb, comp_id, "%s.velocity-fb", b) < 0) return -1;

  // Cache the pin pointers locally for convenience
  *pin_->position_fb = 0.0;
  *pin_->velocity_cmd = 0.0;
  *pin_->position_cmd = 0.0;
  *pin_->velocity_fb = 0.0;

  old_position_cmd_ = 0.0;
  prev_feedback_ = 0;

  return 0;
}

double Stepgen::PositionControl(const double period_s) {
  const double ff_vel = (*pin_->position_cmd - old_position_cmd_) / period_s;
  old_position_cmd_ = *pin_->position_cmd;
  const double velocity_error = *pin_->velocity_fb - ff_vel;
  double match_accel;
  if (velocity_error > 0.0) {
    match_accel = param_->maxaccel == 0 ? -velocity_error / period_s : -param_->maxaccel;
  } else if (velocity_error < 0.0) {
    match_accel = param_->maxaccel == 0 ? velocity_error / period_s : param_->maxaccel;
  } else {
    match_accel = 0;
  }
  const double seconds_to_vel_match = match_accel == 0 ? 0.0 : -velocity_error / match_accel;
  const double position_at_match =
      *pin_->position_fb + (ff_vel + *pin_->velocity_fb) * 0.5 * (seconds_to_vel_match + period_s);
  const double position_cmd_at_match = *pin_->position_cmd + ff_vel * seconds_to_vel_match;
  const double error_at_match = position_at_match - position_cmd_at_match;
  double vel_cmd;
  if (seconds_to_vel_match < period_s) {
    vel_cmd = ff_vel - 0.5 * error_at_match / period_s;
    if (param_->maxaccel > 0) {
      vel_cmd = std::clamp(vel_cmd, *pin_->velocity_fb - param_->maxaccel * period_s,
                           *pin_->velocity_fb + param_->maxaccel * period_s);
    }
  } else {
    if (std::fabs(error_at_match + -4.0 * match_accel * period_s * seconds_to_vel_match) < std::fabs(error_at_match))
      match_accel = -match_accel;
    vel_cmd = *pin_->velocity_fb + match_accel * period_s;
  }
  return vel_cmd;
}

void Stepgen::Update(const double period_s, LinuxCncState *linuxcnc_state) {
  constexpr double max_steps_per_s = static_cast<double>(kStepgenTickFrequency) / 2.0;
  const double physical_maxvel = max_steps_per_s / std::fabs(param_->position_scale);
  if (param_->maxvel < 0.0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "stepgen.%02d.maxvel < 0, setting to abs\n", index_);
    param_->maxvel = std::fabs(param_->maxvel);
  }
  if (param_->maxvel > physical_maxvel) {
    rtapi_print_msg(RTAPI_MSG_ERR, "stepgen.%02d.maxvel too big, clipping to %f\n", index_, physical_maxvel);
    param_->maxvel = physical_maxvel;
  }
  if (param_->maxaccel < 0.0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "stepgen.%02d.maxaccel < 0, setting to abs\n", index_);
    param_->maxaccel = std::fabs(param_->maxaccel);
  }
  const double maxvel = param_->maxvel == 0.0 ? physical_maxvel : param_->maxvel;

  double new_vel;
  if (*pin_->control_type == 0) {
    new_vel = PositionControl(period_s);
  } else {
    new_vel = *pin_->velocity_cmd;
    if (param_->maxaccel > 0.0) {
      if (const double dv = (new_vel - *pin_->velocity_fb) / period_s; dv > param_->maxaccel)
        new_vel = *pin_->velocity_fb + param_->maxaccel * period_s;
      else if (dv < -param_->maxaccel)
        new_vel = *pin_->velocity_fb - param_->maxaccel * period_s;
    }
  }

  new_vel = std::clamp(new_vel, -maxvel, maxvel);

  *pin_->velocity_fb = new_vel;

  const double steps_per_s = new_vel * param_->position_scale;
  if (steps_per_s >= 0.0)
    linuxcnc_state->stepgen_dir_mask |= 1u << index_;
  else
    linuxcnc_state->stepgen_dir_mask &= ~(1u << index_);

  uint64_t steps_per_tick = llround(std::fabs(steps_per_s) * (static_cast<double>(kFixedOne) / kStepgenTickFrequency));
  if (*pin_->enable != 1) steps_per_tick = 0;
  if (steps_per_tick > UINT32_MAX) steps_per_tick = UINT32_MAX;
  linuxcnc_state->steps_per_tick_cmd[index_] = static_cast<uint32_t>(steps_per_tick);
}

void Stepgen::ApplyFeedback(const PruState *pru_state) {
  const int32_t feedback = pru_state->stepgen_feedback[index_];
  const int32_t dd = feedback - prev_feedback_;
  prev_feedback_ = feedback;
  const double dpos = static_cast<double>(dd) / (param_->position_scale * 65536.0);
  *pin_->position_fb = *pin_->position_reset ? 0.0 : *pin_->position_fb + dpos;
}

void Stepgen::ResetPrevFeedback(const PruState *pru_state) { prev_feedback_ = pru_state->stepgen_feedback[index_]; }
