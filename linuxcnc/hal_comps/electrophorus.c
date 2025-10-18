/**
 * Description:  electrophorus.c
 *               A HAL component that provides an SPI connection to a Carvera-family machine
 *               running the Electrophorus PRU firmware.
 *
 *              Initially developed for RaspberryPi -> Arduino Due.
 *              Further developed for RaspberryPi -> Smoothieboard and clones (LPC1768).
 *              Refactored and modified for Carvera-family machines (LPC1768-based controllers).
 *
 * Original Author: Scott Alford
 * Modified by: Konstantin Tcepliaev
 * License: GPL Version 3
 *
 *              Credit to GP Orcullo and PICnc V2 which originally inspired this
 *              and portions of this code is based on stepgen.c by John Kasunich
 *              and hm2_rpspi.c by Matsche
 *
 * Copyright (c) 2024 Scott Alford, All rights reserved.
 * Copyright (c) 2025 Konstantin Tcepliaev <f355@f355.org>, All rights reserved.
 **/

#include <fcntl.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include "hal.h"
#include "rtapi.h"
// ReSharper disable once CppUnusedIncludeDirective
#include "rtapi_app.h"

// SPI drivers
#include "spi/rpi4_spi.c"
#include "spi/rpi5_spi.c"

// data structures for SPI rx/tx
#include "../../src/spi_protocol.h"

#define MODNAME "electrophorus"
#define PREFIX "carvera"

MODULE_AUTHOR("Scott Alford AKA scotta, modified by Konstantin Tcepliaev <f355@f355.org>");
MODULE_DESCRIPTION("Driver for the Carvera family of desktop milling machines");
MODULE_LICENSE("GPL v3");

static int spi_freq = 5000000;
RTAPI_MP_INT(spi_freq, "SPI clock frequency in Hz (default 5,000,000)");

#define f_period_s ((double)(l_period_ns * 1e-9))

typedef struct {
  struct {
    struct {
      hal_float_t *position_cmd;  // in: position command (position units)
      hal_float_t *velocity_cmd;  // in: velocity command
      hal_float_t *position_fb;   // out: position feedback (position units)
      hal_float_t *velocity_fb;   // out: velocity feedback
      hal_bit_t *enable;          // is the stepper enabled?
      hal_bit_t *control_type;    // 0="position control", 1="velocity control"
      hal_bit_t *position_reset;  // reset position when true
    } pin;

    struct {
      hal_float_t position_scale;  // steps per position unit
      hal_float_t maxvel;          // max velocity, (pos units/sec)
      hal_float_t maxaccel;        // max accel (pos units/sec^2)
    } param;
  } hal;

  // this variable holds the previous position command, for
  // computing the feedforward velocity
  hal_float_t old_position_cmd;

  int32_t prev_feedback;

} stepgen_state_t;

typedef struct {
  stepgen_state_t stepgens[STEPGENS];

  hal_bit_t *spi_enable;  // in: are SPI comms enabled?
  hal_bit_t *spi_reset;   // in: should go low when the machine is pulled out of e-stop
  hal_bit_t *spi_status;  // out: will go low if the comms are not working for some reason

  hal_float_t *output_vars[OUTPUT_VARS];  // output variables: PWM controls, etc.
  hal_float_t *input_vars[INPUT_VARS];    // input variables: thermistors, pulse counters, etc.
  hal_bit_t *outputs[OUTPUT_PINS];        // digital output pins
  hal_bit_t *inputs[INPUT_PINS * 2];      // digital input pins, twice for inverted 'not' pins
                                          // passed through to LinuxCNC

  linuxCncState_t linuxcnc_state;
  pruState_t pru_state;

  bool spi_reset_old;
  uint32_t last_packet_seen;
} state_t;

/* other globals */
static int comp_id;  // component ID
static const char *modname = MODNAME;
static const char *prefix = PREFIX;

enum { SPI_UNKNOWN = 0, SPI_RPI4, SPI_RPI5 } spi_driver = SPI_UNKNOWN;

static bool pin_err(int retval);
static int rt_peripheral_init();

static void update_freq(void *arg, long l_period_ns);
static void spi_write(void *arg, long l_period_ns);
static void spi_read(void *arg, long l_period_ns);
static void spi_transfer(state_t *state);

int rtapi_app_main(void) {
  // connect to the HAL, initialise the driver
  comp_id = hal_init(modname);
  if (comp_id < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s ERROR: hal_init() failed \n", modname);
    return -1;
  }

  // allocate shared memory
  state_t *state = hal_malloc(sizeof(state_t));
  if (state == 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed\n", modname);
    hal_exit(comp_id);
    return -1;
  }

  state->spi_reset_old = false;
  state->last_packet_seen = 0;

  if (rt_peripheral_init() != 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "rt_peripheral_init failed.\n");
    return -1;
  }

  if (pin_err(hal_pin_bit_newf(HAL_IN, &state->spi_enable, comp_id, "%s.spi-enable", prefix))) return -1;
  if (pin_err(hal_pin_bit_newf(HAL_IN, &state->spi_reset, comp_id, "%s.spi-reset", prefix))) return -1;
  if (pin_err(hal_pin_bit_newf(HAL_OUT, &state->spi_status, comp_id, "%s.spi-status", prefix))) return -1;

  // export all the variables for each stepper and pin
  for (int i = 0; i < STEPGENS; i++) {
    stepgen_state_t *s = &state->stepgens[i];

    char *stepper_names[STEPGENS] = STEPGEN_NAMES;
    char *name = stepper_names[i];

    typeof(s->hal.param) *par = &s->hal.param;

    if (pin_err(
            hal_param_float_newf(HAL_RW, &par->position_scale, comp_id, "%s.stepgen.%s.position-scale", prefix, name)))
      return -1;
    par->position_scale = 1.0;

    if (pin_err(hal_param_float_newf(HAL_RW, &par->maxvel, comp_id, "%s.stepgen.%s.maxvel", prefix, name))) return -1;
    par->maxvel = 0.0;

    if (pin_err(hal_param_float_newf(HAL_RW, &par->maxaccel, comp_id, "%s.stepgen.%s.maxaccel", prefix, name)))
      return -1;
    par->maxaccel = 1.0;

    typeof(s->hal.pin) *pin = &s->hal.pin;

    if (pin_err(hal_pin_bit_newf(HAL_IN, &pin->enable, comp_id, "%s.stepgen.%s.enable", prefix, name))) return -1;

    if (pin_err(hal_pin_bit_newf(HAL_IN, &pin->control_type, comp_id, "%s.stepgen.%s.control-type", prefix, name)))
      return -1;

    if (pin_err(hal_pin_bit_newf(HAL_IN, &pin->position_reset, comp_id, "%s.stepgen.%s.position-reset", prefix, name)))
      return -1;

    if (pin_err(hal_pin_float_newf(HAL_IN, &pin->position_cmd, comp_id, "%s.stepgen.%s.position-cmd", prefix, name)))
      return -1;
    *pin->position_cmd = 0.0;

    if (pin_err(hal_pin_float_newf(HAL_IN, &pin->velocity_cmd, comp_id, "%s.stepgen.%s.velocity-cmd", prefix, name)))
      return -1;
    *pin->velocity_cmd = 0.0;

    if (pin_err(hal_pin_float_newf(HAL_OUT, &pin->position_fb, comp_id, "%s.stepgen.%s.position-fb", prefix, name)))
      return -1;
    *pin->position_fb = 0.0;

    if (pin_err(hal_pin_float_newf(HAL_OUT, &pin->velocity_fb, comp_id, "%s.stepgen.%s.velocity-fb", prefix, name)))
      return -1;
    *pin->velocity_fb = 0.0;
  }

  for (int i = 0; i < OUTPUT_VARS; i++) {
    const char *output_var_names[OUTPUT_VARS] = OUTPUT_VAR_NAMES;
    if (pin_err(hal_pin_float_newf(HAL_IN, &state->output_vars[i], comp_id, "%s.output-var.%s", prefix,
                                   output_var_names[i])))
      return -1;
    *state->output_vars[i] = 0;
  }

  for (int i = 0; i < INPUT_VARS; i++) {
    const char *input_var_names[INPUT_VARS] = INPUT_VAR_NAMES;
    if (pin_err(
            hal_pin_float_newf(HAL_OUT, &state->input_vars[i], comp_id, "%s.input-var.%s", prefix, input_var_names[i])))
      return -1;
    *state->input_vars[i] = 0;
  }

  for (int i = 0; i < OUTPUT_PINS; i++) {
    const outputPin_t output_pins[OUTPUT_PINS] = OUTPUT_PIN_DESC;
    if (pin_err(hal_pin_bit_newf(HAL_IN, &state->outputs[i], comp_id, "%s.output.%s", prefix, output_pins[i].name)))
      return -1;
    *state->outputs[i] = 0;
  }

  for (int i = 0; i < INPUT_PINS; i++) {
    const inputPin_t input_pins[INPUT_PINS] = INPUT_PIN_DESC;
    if (pin_err(hal_pin_bit_newf(HAL_OUT, &state->inputs[i], comp_id, "%s.input.%s", prefix, input_pins[i].name)))
      return -1;
    *state->inputs[i] = 0;

    // inverted 'not' pins offset by the number of inputs we have.
    if (pin_err(hal_pin_bit_newf(HAL_OUT, &state->inputs[i + INPUT_PINS], comp_id, "%s.input.%s.not", prefix,
                                 input_pins[i].name)))
      return -1;
    *state->inputs[i + INPUT_PINS] = 0;
  }

  // Export functions
  char name[HAL_NAME_LEN + 1];
  rtapi_snprintf(name, sizeof(name), "%s.update-freq", prefix);
  int retval = hal_export_funct(name, update_freq, state, 1, 0, comp_id);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: update function export failed\n", modname);
    hal_exit(comp_id);
    return -1;
  }

  rtapi_snprintf(name, sizeof(name), "%s.write", prefix);
  /* no FP operations */
  retval = hal_export_funct(name, spi_write, state, 0, 0, comp_id);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: write function export failed\n", modname);
    hal_exit(comp_id);
    return -1;
  }

  rtapi_snprintf(name, sizeof(name), "%s.read", prefix);
  retval = hal_export_funct(name, spi_read, state, 1, 0, comp_id);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: read function export failed\n", modname);
    hal_exit(comp_id);
    return -1;
  }

  rtapi_print_msg(RTAPI_MSG_INFO, "%s: installed driver\n", modname);
  hal_ready(comp_id);
  return 0;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }

bool pin_err(const int retval) {
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin export failed with err=%i\n", modname, retval);
    hal_exit(comp_id);
    return true;
  }
  return false;
}

static double stepgen_instance_position_control(stepgen_state_t *s, const long l_period_ns) {
  // calculate feed-forward velocity in machine units per second
  const double ff_vel = (*s->hal.pin.position_cmd - s->old_position_cmd) / f_period_s;

  s->old_position_cmd = *s->hal.pin.position_cmd;

  const double velocity_error = *s->hal.pin.velocity_fb - ff_vel;

  // Do we need to change speed to match the speed of position-cmd?
  // If maxaccel is 0, there's no accel limit: fix this velocity error
  // by the next servo period!  This leaves acceleration control up to
  // the trajectory planner.
  // If maxaccel is not zero, the user has specified a maxaccel and we
  // adhere to that.
  double match_accel;
  if (velocity_error > 0.0) {
    if (s->hal.param.maxaccel == 0) {
      match_accel = -velocity_error / f_period_s;
    } else {
      match_accel = -s->hal.param.maxaccel;
    }
  } else if (velocity_error < 0.0) {
    if (s->hal.param.maxaccel == 0) {
      match_accel = velocity_error / f_period_s;
    } else {
      match_accel = s->hal.param.maxaccel;
    }
  } else {
    match_accel = 0;
  }

  double seconds_to_vel_match;
  if (match_accel == 0) {
    // vel is just right, dont need to accelerate
    seconds_to_vel_match = 0.0;
  } else {
    seconds_to_vel_match = -velocity_error / match_accel;
  }
  //*s->hal.pin.dbg_s_to_match = seconds_to_vel_match;

  // compute expected position at the time of velocity match
  // Note: this is "feedback position at the beginning of the servo period after we attain velocity match"
  double position_at_match;
  {
    const double avg_v = (ff_vel + *s->hal.pin.velocity_fb) * 0.5;
    position_at_match = *s->hal.pin.position_fb + avg_v * (seconds_to_vel_match + f_period_s);
  }

  // Note: this assumes that position-cmd keeps the current velocity
  const double position_cmd_at_match = *s->hal.pin.position_cmd + ff_vel * seconds_to_vel_match;
  const double error_at_match = position_at_match - position_cmd_at_match;

  //*s->hal.pin.dbg_err_at_match = error_at_match;

  double velocity_cmd;
  if (seconds_to_vel_match < f_period_s) {
    // we can match velocity in one period
    // try to correct whatever position error we have
    velocity_cmd = ff_vel - (0.5 * error_at_match / f_period_s);

    // apply accel limits?
    if (s->hal.param.maxaccel > 0) {
      if (velocity_cmd > *s->hal.pin.velocity_fb + s->hal.param.maxaccel * f_period_s) {
        velocity_cmd = *s->hal.pin.velocity_fb + s->hal.param.maxaccel * f_period_s;
      } else if (velocity_cmd < *s->hal.pin.velocity_fb - s->hal.param.maxaccel * f_period_s) {
        velocity_cmd = *s->hal.pin.velocity_fb - s->hal.param.maxaccel * f_period_s;
      }
    }

  } else {
    // we're going to have to work for more than one period to match velocity
    // FIXME: I dont really get this part yet

    // calculate change in final position if we ramp in the opposite direction for one period
    const double dv = -2.0 * match_accel * f_period_s;
    const double dp = dv * seconds_to_vel_match;

    // decide which way to ramp
    if (fabs(error_at_match + dp * 2.0) < fabs(error_at_match)) {
      match_accel = -match_accel;
    }

    // and do it
    velocity_cmd = *s->hal.pin.velocity_fb + match_accel * f_period_s;
  }

  return velocity_cmd;
}

void update_freq(void *arg, const long l_period_ns) {
  state_t *state = arg;

  // recompute dir mask from scratch each cycle
  state->linuxcnc_state.stepgen_dir_mask = 0;

  // loop through generators
  for (int i = 0; i < STEPGENS; i++) {
    stepgen_state_t *s = &state->stepgens[i];

    // first sanity-check our maxaccel and maxvel params
    // maxvel must be >= 0.0, and may not be faster than 1 step per (steplen+stepspace) seconds

    const double max_steps_per_s = (double)STEPGEN_TICK_FREQUENCY / 2.0;

    // max vel supported by current step timings & position-scale
    const double physical_maxvel = max_steps_per_s / fabs(s->hal.param.position_scale);
    // physical_maxvel = force_precision(physical_maxvel);

    if (s->hal.param.maxvel < 0.0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "stepgen.%02d.maxvel < 0, setting to its absolute value\n", i);
      s->hal.param.maxvel = fabs(s->hal.param.maxvel);
    }

    if (s->hal.param.maxvel > physical_maxvel) {
      rtapi_print_msg(
          RTAPI_MSG_ERR,
          "stepgen.%02d.maxvel is too big for current step timings (%f steps/s) & position-scale (%f), clipping to "
          "max possible (%f)\n",
          i, max_steps_per_s, s->hal.param.position_scale, physical_maxvel);
      s->hal.param.maxvel = physical_maxvel;
    }

    double maxvel;  // actual max vel to use this time

    if (s->hal.param.maxvel == 0.0) {
      maxvel = physical_maxvel;
    } else {
      maxvel = s->hal.param.maxvel;
    }

    // maxaccel may not be negative
    if (s->hal.param.maxaccel < 0.0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "stepgen.%02d.maxaccel < 0, setting to its absolute value\n", i);
      s->hal.param.maxaccel = fabs(s->hal.param.maxaccel);
    }

    double new_vel;
    // select the new velocity we want
    if (*s->hal.pin.control_type == 0) {
      new_vel = stepgen_instance_position_control(s, l_period_ns);
    } else {
      // velocity-mode control is easy
      new_vel = *s->hal.pin.velocity_cmd;
      if (s->hal.param.maxaccel > 0.0) {
        if (((new_vel - *s->hal.pin.velocity_fb) / f_period_s) > s->hal.param.maxaccel) {
          new_vel = (*s->hal.pin.velocity_fb) + (s->hal.param.maxaccel * f_period_s);
        } else if (((new_vel - *s->hal.pin.velocity_fb) / f_period_s) < -s->hal.param.maxaccel) {
          new_vel = (*s->hal.pin.velocity_fb) - (s->hal.param.maxaccel * f_period_s);
        }
      }
    }

    // clip velocity to maxvel
    if (new_vel > maxvel) {
      new_vel = maxvel;
    } else if (new_vel < -maxvel) {
      new_vel = -maxvel;
    }

    *s->hal.pin.velocity_fb = (hal_float_t)new_vel;

    const double steps_per_s = new_vel * s->hal.param.position_scale;

    // Set direction bit from the sign of velocity (bit i = forward)
    if (steps_per_s >= 0.0) {
      state->linuxcnc_state.stepgen_dir_mask |= 1u << i;
    } else {
      state->linuxcnc_state.stepgen_dir_mask &= ~(1u << i);
    }

    uint64_t steps_per_tick = (uint64_t)llround(fabs(steps_per_s) * ((double)FIXED_ONE / STEPGEN_TICK_FREQUENCY));
    if (*s->hal.pin.enable != 1) steps_per_tick = 0;  // disabled axis => zero steps/tick
    if (steps_per_tick > UINT32_MAX) steps_per_tick = UINT32_MAX;
    state->linuxcnc_state.steps_per_tick_cmd[i] = (uint32_t)steps_per_tick;
  }
}

void spi_read(void *arg, const long l_period_ns) {
  state_t *state = arg;
  (void)l_period_ns;

  // detect reset rising edge (happens when the machine is pulled out of e-stop)
  const bool reset_edge = (*state->spi_reset && !state->spi_reset_old);
  state->spi_reset_old = *state->spi_reset;

  if (!*state->spi_enable || (!reset_edge && !*state->spi_status)) {
    // we're either in e-stop completely, or the comms are not up and we're not trying to come out of e-stop (i.e.
    // something is wrong)
    *state->spi_status = 0;
    return;
  }

  state->linuxcnc_state.command = PRU_READ;

  // Transfer to and from the PRU
  spi_transfer(state);

  const uint32_t cnt = state->pru_state.packet_counter;

  bool reset_feedback = false;
  bool publish_inputs = false;

  if (reset_edge) {
    // we're out of estop, accept any counter
    if (state->last_packet_seen != 0 && cnt == PRU_INIT_PKT) {
      rtapi_print_msg(RTAPI_MSG_ERR,
                      "PRU has been reset while LinuxCNC was in e-stop. If the axes were moved by hand, "
                      "you need to un-home and re-home the machine.\n");
    }
    *state->spi_status = 1;
    reset_feedback = true;
    publish_inputs = true;
  } else if (state->last_packet_seen + 1u == cnt) {
    // in normal operation, the packet counter steadily increments
    *state->spi_status = 1;
    for (int i = 0; i < STEPGENS; i++) {
      stepgen_state_t *s = &state->stepgens[i];
      const int32_t feedback = state->pru_state.stepgen_feedback[i];

      const int32_t dd = feedback - s->prev_feedback;
      s->prev_feedback = feedback;
      const double dpos = (double)dd / (s->hal.param.position_scale * 65536.0);
      *s->hal.pin.position_fb = *s->hal.pin.position_reset != 0 ? 0.0 : (*s->hal.pin.position_fb + dpos);
    }
    publish_inputs = true;
  } else {
    // packet counter mismatch, PRU is misbehaving; disable comms and drop into e-stop
    *state->spi_status = 0;
    reset_feedback = true;
  }

  state->last_packet_seen = cnt;

  if (reset_feedback) {
    for (int i = 0; i < STEPGENS; i++) {
      state->stepgens[i].prev_feedback = state->pru_state.stepgen_feedback[i];
    }
  }

  if (publish_inputs) {
    for (int i = 0; i < INPUT_VARS; i++) {
      *state->input_vars[i] = state->pru_state.input_vars[i];
    }
    for (int i = 0; i < INPUT_PINS; i++) {
      if ((state->pru_state.inputs & (1 << i)) != 0) {
        *state->inputs[i] = 1;
        *state->inputs[i + INPUT_PINS] = 0;
      } else {
        *state->inputs[i] = 0;
        *state->inputs[i + INPUT_PINS] = 1;
      }
    }
  }
}

void spi_write(void *arg, const long l_period_ns) {
  state_t *state = arg;
  (void)l_period_ns;

  int i;

  state->linuxcnc_state.command = PRU_WRITE;

  for (i = 0; i < OUTPUT_VARS; i++) {
    state->linuxcnc_state.output_vars[i] = (int32_t)*state->output_vars[i];
  }

  for (i = 0; i < OUTPUT_PINS; i++) {
    if (*state->outputs[i] == 1) {
      state->linuxcnc_state.outputs |= 1 << i;
    } else {
      state->linuxcnc_state.outputs &= ~(1 << i);
    }
  }

  if (*state->spi_status) {
    spi_transfer(state);
  }
}

void spi_transfer(state_t *state) {
  switch (spi_driver) {
    case SPI_RPI4:
      rpi4_spi_xfer(state->pru_state.buffer, state->linuxcnc_state.buffer, SPI_BUF_SIZE);
      break;
    case SPI_RPI5:
      rpi5_spi_xfer(state->pru_state.buffer, state->linuxcnc_state.buffer, SPI_BUF_SIZE);
      break;
    default:
      rtapi_print_msg(RTAPI_MSG_ERR, "unknown SPI driver\n");
  }
}

int rt_peripheral_init(void) {
  int ret5 = rpi5_spi_init(spi_freq);
  if (ret5 == 1) {
    rtapi_print_msg(RTAPI_MSG_INFO, "Raspberry Pi 5, using rpi5 spi driver\n");
    spi_driver = SPI_RPI5;
    return 0;
  }

  int ret4 = rpi4_spi_init(spi_freq);
  if (ret4 == 1) {
    rtapi_print_msg(RTAPI_MSG_INFO, "Raspberry Pi 4, using rpi4 spi driver\n");
    spi_driver = SPI_RPI4;
    return 0;
  }

  if (ret5 < 0) rtapi_print_msg(RTAPI_MSG_ERR, "rpi5_spi_init failed (%d).\n", ret5);
  if (ret4 < 0) rtapi_print_msg(RTAPI_MSG_ERR, "rpi4_spi_init failed (%d).\n", ret4);
  rtapi_print_msg(RTAPI_MSG_ERR, "Error: no SPI driver available (need root/CAP_SYS_RAWIO for /dev/mem).\n");
  return -1;
}
