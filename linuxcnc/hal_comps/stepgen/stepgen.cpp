#include "stepgen.hpp"

#include <algorithm>
#include <cmath>
#include <string>

int Stepgen::init(const int comp_id, const char *const modname, const char *const prefix, const int i) {
  this->index = i;
  param = static_cast<Params *>(hal_malloc(sizeof(Params)));
  if (!param) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed for stepgen params\n", modname);
    return -1;
  }
  pin = static_cast<Pins *>(hal_malloc(sizeof(Pins)));
  if (!pin) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed for stepgen pin pointers\n", modname);
    return -1;
  }

  const char *stepper_names[STEPGENS] = STEPGEN_NAMES;
  const std::string base = std::string(prefix) + ".stepgen." + stepper_names[i];
  const char *b = base.c_str();

  // Params (storage must live in HAL shared memory)
  if (hal_param_float_newf(HAL_RW, &param->position_scale, comp_id, "%s.position-scale", b) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: stepgen param export failed (position-scale)\n", modname);
    return -1;
  }
  if (hal_param_float_newf(HAL_RW, &param->maxvel, comp_id, "%s.maxvel", b) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: stepgen param export failed (maxvel)\n", modname);
    return -1;
  }
  if (hal_param_float_newf(HAL_RW, &param->maxaccel, comp_id, "%s.maxaccel", b) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: stepgen param export failed (maxaccel)\n", modname);
    return -1;
  }

  param->position_scale = 1.0;
  param->maxvel = 0.0;
  param->maxaccel = 1.0;

  // Pins - pointer variables themselves must be in HAL shared memory
  if (hal_pin_bit_newf(HAL_IN, &pin->enable, comp_id, "%s.enable", b) < 0) return -1;
  if (hal_pin_bit_newf(HAL_IN, &pin->control_type, comp_id, "%s.control-type", b) < 0) return -1;
  if (hal_pin_bit_newf(HAL_IN, &pin->position_reset, comp_id, "%s.position-reset", b) < 0) return -1;
  if (hal_pin_float_newf(HAL_IN, &pin->position_cmd, comp_id, "%s.position-cmd", b) < 0) return -1;
  if (hal_pin_float_newf(HAL_IN, &pin->velocity_cmd, comp_id, "%s.velocity-cmd", b) < 0) return -1;
  if (hal_pin_float_newf(HAL_OUT, &pin->position_fb, comp_id, "%s.position-fb", b) < 0) return -1;
  if (hal_pin_float_newf(HAL_OUT, &pin->velocity_fb, comp_id, "%s.velocity-fb", b) < 0) return -1;

  // Cache the pin pointers locally for convenience
  *pin->position_fb = 0.0;
  *pin->velocity_cmd = 0.0;
  *pin->position_cmd = 0.0;
  *pin->velocity_fb = 0.0;

  old_position_cmd = 0.0;
  prev_feedback = 0;

  return 0;
}

double Stepgen::position_control(const double period_s) {
  const double ff_vel = (*pin->position_cmd - old_position_cmd) / period_s;
  old_position_cmd = *pin->position_cmd;
  const double velocity_error = *pin->velocity_fb - ff_vel;
  double match_accel;
  if (velocity_error > 0.0) {
    match_accel = param->maxaccel == 0 ? -velocity_error / period_s : -param->maxaccel;
  } else if (velocity_error < 0.0) {
    match_accel = param->maxaccel == 0 ? velocity_error / period_s : param->maxaccel;
  } else {
    match_accel = 0;
  }
  const double seconds_to_vel_match = match_accel == 0 ? 0.0 : -velocity_error / match_accel;
  const double position_at_match =
      *pin->position_fb + (ff_vel + *pin->velocity_fb) * 0.5 * (seconds_to_vel_match + period_s);
  const double position_cmd_at_match = *pin->position_cmd + ff_vel * seconds_to_vel_match;
  const double error_at_match = position_at_match - position_cmd_at_match;
  double vel_cmd;
  if (seconds_to_vel_match < period_s) {
    vel_cmd = ff_vel - 0.5 * error_at_match / period_s;
    if (param->maxaccel > 0) {
      vel_cmd = std::clamp(vel_cmd, *pin->velocity_fb - param->maxaccel * period_s,
                           *pin->velocity_fb + param->maxaccel * period_s);
    }
  } else {
    if (std::fabs(error_at_match + -4.0 * match_accel * period_s * seconds_to_vel_match) < std::fabs(error_at_match))
      match_accel = -match_accel;
    vel_cmd = *pin->velocity_fb + match_accel * period_s;
  }
  return vel_cmd;
}

void Stepgen::update(const double period_s, LinuxCncState *linuxcnc_state) {
  constexpr double max_steps_per_s = static_cast<double>(STEPGEN_TICK_FREQUENCY) / 2.0;
  const double physical_maxvel = max_steps_per_s / std::fabs(param->position_scale);
  if (param->maxvel < 0.0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "stepgen.%02d.maxvel < 0, setting to abs\n", index);
    param->maxvel = std::fabs(param->maxvel);
  }
  if (param->maxvel > physical_maxvel) {
    rtapi_print_msg(RTAPI_MSG_ERR, "stepgen.%02d.maxvel too big, clipping to %f\n", index, physical_maxvel);
    param->maxvel = physical_maxvel;
  }
  if (param->maxaccel < 0.0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "stepgen.%02d.maxaccel < 0, setting to abs\n", index);
    param->maxaccel = std::fabs(param->maxaccel);
  }
  const double maxvel = param->maxvel == 0.0 ? physical_maxvel : param->maxvel;

  double new_vel;
  if (*pin->control_type == 0) {
    new_vel = position_control(period_s);
  } else {
    new_vel = *pin->velocity_cmd;
    if (param->maxaccel > 0.0) {
      if (const double dv = (new_vel - *pin->velocity_fb) / period_s; dv > param->maxaccel)
        new_vel = *pin->velocity_fb + param->maxaccel * period_s;
      else if (dv < -param->maxaccel)
        new_vel = *pin->velocity_fb - param->maxaccel * period_s;
    }
  }

  new_vel = std::clamp(new_vel, -maxvel, maxvel);

  *pin->velocity_fb = new_vel;

  const double steps_per_s = new_vel * param->position_scale;
  if (steps_per_s >= 0.0)
    linuxcnc_state->stepgen_dir_mask |= 1u << index;
  else
    linuxcnc_state->stepgen_dir_mask &= ~(1u << index);

  uint64_t steps_per_tick = llround(std::fabs(steps_per_s) * (static_cast<double>(FIXED_ONE) / STEPGEN_TICK_FREQUENCY));
  if (*pin->enable != 1) steps_per_tick = 0;
  if (steps_per_tick > UINT32_MAX) steps_per_tick = UINT32_MAX;
  linuxcnc_state->steps_per_tick_cmd[index] = static_cast<uint32_t>(steps_per_tick);
}

void Stepgen::apply_feedback(const PruState *pru_state) {
  const int32_t feedback = pru_state->stepgen_feedback[index];
  const int32_t dd = feedback - prev_feedback;
  prev_feedback = feedback;
  const double dpos = static_cast<double>(dd) / (param->position_scale * 65536.0);
  *pin->position_fb = *pin->position_reset ? 0.0 : *pin->position_fb + dpos;
}

void Stepgen::reset_prev_feedback(const PruState *pru_state) { prev_feedback = pru_state->stepgen_feedback[index]; }
