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
#include "rtapi_app.h"

// Using BCM2835 driver library by Mike McCauley, why reinvent the wheel!
// http://www.airspayce.com/mikem/bcm2835/index.html
// Include these in the source directory when using "halcompile --install electrophorus.c"
#include "bcm2835.c"
#include "bcm2835.h"

// Raspberry Pi 5 uses the RP1
#include "dtcboards.h"
#include "gpiochip_rp1.c"
#include "gpiochip_rp1.h"
#include "rp1lib.c"
#include "rp1lib.h"
#include "spi-dw.c"
#include "spi-dw.h"

// data structures for SPI rx/tx
#include "spi_data.h"

#define MODNAME "electrophorus"
#define PREFIX "carvera"

MODULE_AUTHOR("Scott Alford AKA scotta, modified by Konstantin Tcepliaev <f355@f355.org>");
MODULE_DESCRIPTION("Driver for the Carvera family of desktop milling machines");
MODULE_LICENSE("GPL v3");

#define RPI5_RP1_PERI_BASE 0x7c000000

#define f_period_s ((double)(l_period_ns * 1e-9))

typedef struct {
  struct {
    struct {
      hal_float_t *position_cmd;  // in: position command (position units)
      hal_float_t *velocity_cmd;  // in: velocity command
      hal_s32_t *counts;          // out: position feedback (raw counts)
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

  fixp_t prev_accumulator;
  int64_t subcounts;
} stepper_state_t;

typedef struct {
  stepper_state_t stepgens[STEPGENS];

  hal_bit_t *spi_enable;  // in: are SPI comms enabled?
  hal_bit_t *spi_reset;   // in: should go low when the machine is pulled out of e-stop
  hal_bit_t *spi_status;  // out: will go low if the comms are not working for some reason

  hal_float_t *output_vars[OUTPUT_VARS];  // output variables: PWM controls, etc.
  hal_float_t *input_vars[INPUT_VARS];    // input variables: thermistors, pulse counters, etc.
  hal_bit_t *outputs[OUTPUT_PINS];        // digital output pins
  hal_bit_t *inputs[INPUT_PINS * 2];      // digital input pins, twice for inverted 'not' pins
                                          // passed through to LinuxCNC

  bool spi_reset_old;
} state_t;

static state_t *state;

typedef pruData_t rxData_t;
typedef linuxCncData_t txData_t;

static txData_t tx_data;
static rxData_t rx_data;

/* other globals */
static int comp_id;  // component ID
static const char *modname = MODNAME;
static const char *prefix = PREFIX;

enum { DRV_UNKNOWN = 0, DRV_BCM, DRV_RP1 } spi_driver = DRV_UNKNOWN;

static bool pin_err(int retval);
static int rt_peripheral_init();
static int rt_bcm2835_init();
static int rt_rp1lib_init();

static void update_freq(void *arg, long l_period_ns);
static void spi_write();
static void spi_read();
static void spi_transfer();

int rtapi_app_main(void) {
  // connect to the HAL, initialise the driver
  comp_id = hal_init(modname);
  if (comp_id < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s ERROR: hal_init() failed \n", modname);
    return -1;
  }

  // allocate shared memory
  state = hal_malloc(sizeof(state_t));
  if (state == 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed\n", modname);
    hal_exit(comp_id);
    return -1;
  }

  if (rt_peripheral_init() != 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "rt_peripheral_init failed.\n");
    return -1;
  }

  if (pin_err(hal_pin_bit_newf(HAL_IN, &state->spi_enable, comp_id, "%s.spi-enable", prefix))) return -1;
  if (pin_err(hal_pin_bit_newf(HAL_IN, &state->spi_reset, comp_id, "%s.spi-reset", prefix))) return -1;
  if (pin_err(hal_pin_bit_newf(HAL_OUT, &state->spi_status, comp_id, "%s.spi-status", prefix))) return -1;

  // export all the variables for each stepper and pin
  for (int i = 0; i < STEPGENS; i++) {
    stepper_state_t *s = &state->stepgens[i];

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

    if (pin_err(hal_pin_s32_newf(HAL_OUT, &pin->counts, comp_id, "%s.stepgen.%s.counts", prefix, name))) return -1;
    *pin->counts = 0;
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
  retval = hal_export_funct(name, spi_write, 0, 0, 0, comp_id);
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

static void stepgen_instance_position_control(stepper_state_t *s, const long l_period_ns, double *new_vel) {
  //(*s->hal.pin.dbg_pos_minus_prev_cmd) = (*s->hal.pin.position_fb) - s->old_position_cmd;

  // calculate feed-forward velocity in machine units per second
  const double ff_vel = (*s->hal.pin.position_cmd - s->old_position_cmd) / f_period_s;
  //(*s->hal.pin.dbg_ff_vel) = ff_vel;

  s->old_position_cmd = *s->hal.pin.position_cmd;

  const double velocity_error = *s->hal.pin.velocity_fb - ff_vel;
  // (*s->hal.pin.dbg_vel_error) = velocity_error;

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
    double avg_v = (ff_vel + *s->hal.pin.velocity_fb) * 0.5;
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

  *new_vel = velocity_cmd;
}

void update_freq(void *arg, const long l_period_ns) {
  state_t *state = arg;

  // loop through generators
  for (int i = 0; i < STEPGENS; i++) {
    stepper_state_t *s = &state->stepgens[i];

    // first sanity-check our maxaccel and maxvel params
    // maxvel must be >= 0.0, and may not be faster than 1 step per (steplen+stepspace) seconds

    const double max_steps_per_s = (double)BASE_FREQUENCY / 2.0;

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
      stepgen_instance_position_control(s, l_period_ns, &new_vel);
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

    tx_data.stepgen_freq_command[i] = (fixp_t)(new_vel * s->hal.param.position_scale * FIXED_ONE);
  }
}

void spi_read() {
  // Data header
  tx_data.header = PRU_READ;

  if (*(state->spi_enable)) {
    if ((*state->spi_reset && !state->spi_reset_old) || *state->spi_status) {
      // reset rising edge detected, try SPI transfer and reset OR PRU running

      // Transfer to and from the PRU
      spi_transfer();

      switch (rx_data.header)  // only process valid SPI payloads. This rejects bad payloads
      {
        case PRU_DATA:
          // we have received a GOOD payload from the PRU
          *state->spi_status = 1;

          for (int i = 0; i < STEPGENS; i++) {
            stepper_state_t *s = &state->stepgens[i];

            const fixp_t acc = rx_data.stepgen_feedback[i];

            // those tricky users are always trying to get us to divide by zero
            if (fabs(s->hal.param.position_scale) < 1e-6) {
              if (s->hal.param.position_scale >= 0.0) {
                s->hal.param.position_scale = 1.0;
                rtapi_print_msg(RTAPI_MSG_ERR, "stepgen %d position_scale is too close to 0, resetting to 1.0\n", i);
              } else {
                s->hal.param.position_scale = -1.0;
                rtapi_print_msg(RTAPI_MSG_ERR, "stepgen %d position_scale is too close to 0, resetting to -1.0\n", i);
              }
            }

            // The accumulator is a 32.32 bit fixed-point
            // representation of the current stepper position.
            // The fractional part gives accurate velocity at low speeds, and
            // sub-step position feedback (like sw stepgen).
            fixp_t acc_delta = acc - s->prev_accumulator;
            if (acc_delta > INT64_MAX) {
              acc_delta -= UINT64_MAX;
            } else if (acc_delta < INT64_MIN) {
              acc_delta += UINT64_MAX;
            }

            s->subcounts += acc_delta;

            if (*s->hal.pin.position_reset != 0) {
              s->subcounts = 0;
            }

            *s->hal.pin.counts = (int32_t)(s->subcounts / FIXED_ONE);

            // note that it's important to use "subcounts/(1 << STEP_BIT)" instead of just
            // "counts" when computing position_fb, because position_fb needs sub-count precision
            *s->hal.pin.position_fb = (double)s->subcounts / FIXED_ONE / s->hal.param.position_scale;
            s->prev_accumulator = acc;
          }

          for (int i = 0; i < INPUT_VARS; i++) {
            *state->input_vars[i] = rx_data.input_vars[i];
          }

          // Inputs
          for (int i = 0; i < INPUT_PINS; i++) {
            if ((rx_data.inputs & (1 << i)) != 0) {
              *state->inputs[i] = 1;               // input is high
              *state->inputs[i + INPUT_PINS] = 0;  // inverted 'not' is offset by number of digital inputs.
            } else {
              *state->inputs[i] = 0;               // input is low
              *state->inputs[i + INPUT_PINS] = 1;  // inverted 'not' is offset by number of digital inputs.
            }
          }
          break;

        default:
          // we have received a BAD payload from the PRU
          *state->spi_status = 0;
          rtapi_print("Bad SPI payload:");
          for (int i = 0; i < SPI_BUF_SIZE; i++) {
            rtapi_print(" %02x", rx_data.buffer[i]);
          }
          rtapi_print("\n");
      }
    }
  } else {
    *state->spi_status = 0;
  }

  state->spi_reset_old = *state->spi_reset;
}

void spi_write() {
  int i;

  tx_data.header = PRU_WRITE;

  for (i = 0; i < STEPGENS; i++) {
    const stepper_state_t *s = &state->stepgens[i];
    if (*s->hal.pin.enable == 1) {
      tx_data.stepgen_enable_mask |= 1 << i;
    } else {
      tx_data.stepgen_enable_mask &= ~(1 << i);
    }
  }

  for (i = 0; i < OUTPUT_VARS; i++) {
    tx_data.output_vars[i] = (int32_t)*state->output_vars[i];
  }

  for (i = 0; i < OUTPUT_PINS; i++) {
    if (*state->outputs[i] == 1) {
      tx_data.outputs |= 1 << i;
    } else {
      tx_data.outputs &= ~(1 << i);
    }
  }

  if (*state->spi_status) {
    spi_transfer();
  }
}

void spi_transfer() {
  switch (spi_driver) {
    case DRV_BCM:
      // TODO(f355): why transfer byte by byte?
      // bcm2835_spi_transfernb(tx_data.buffer, rx_data.buffer, SPI_BUF_SIZE);
      for (int i = 0; i < SPI_BUF_SIZE; i++) {
        rx_data.buffer[i] = bcm2835_spi_transfer(tx_data.buffer[i]);
      }
      break;
    case DRV_RP1:
      for (int i = 0; i < SPI_BUF_SIZE; i++) {
        rp1spi_transfer(0, tx_data.buffer + i, rx_data.buffer + i, 1);
      }
      break;
    default:
      rtapi_print_msg(RTAPI_MSG_ERR, "unknown SPI driver\n");
  }
}

int rt_peripheral_init(void) {
  char buf[256];
  const int DTC_MAX = 8;
  const char *dtcs[DTC_MAX + 1];

  // assume were only running on >RPi3

  FILE *fp = fopen("/proc/device-tree/compatible", "rb");
  if (!fp) {
    rtapi_print_msg(RTAPI_MSG_ERR, "Cannot open '/proc/device-tree/compatible' for read.\n");
    return -1;
  }

  // Read the 'compatible' string-list from the device-tree
  const size_t buflen = fread(buf, 1, sizeof(buf), fp);
  if (buflen == 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "Failed to read platform identity.\n");
    return -1;
  }
  fclose(fp);

  // Decompose the device-tree buffer into a string-list with the pointers to
  // each string in dtcs. Don't go beyond the buffer's size.
  memset(dtcs, 0, sizeof(dtcs));
  char *cptr = buf;
  for (int i = 0; i < DTC_MAX && cptr; i++) {
    dtcs[i] = cptr;
    const size_t j = strlen(cptr);
    if ((cptr - buf) + j + 1 < buflen)
      cptr += j + 1;
    else
      cptr = NULL;
  }

  for (int i = 0; dtcs[i] != NULL; i++) {
    if (!strcmp(dtcs[i], DTC_RPI_MODEL_4B) || !strcmp(dtcs[i], DTC_RPI_MODEL_4CM) ||
        !strcmp(dtcs[i], DTC_RPI_MODEL_400)) {
      rtapi_print_msg(RTAPI_MSG_INFO, "Raspberry Pi 4, using BCM2835 driver\n");
      spi_driver = DRV_BCM;
      break;  // Found our supported board
    }
    if (!strcmp(dtcs[i], DTC_RPI_MODEL_5B) || !strcmp(dtcs[i], DTC_RPI_MODEL_5CM)) {
      rtapi_print_msg(RTAPI_MSG_INFO, "Raspberry Pi 5, using rp1 driver\n");
      spi_driver = DRV_RP1;
      break;  // Found our supported board
    }
  }

  if (spi_driver == DRV_UNKNOWN) {
    rtapi_print_msg(RTAPI_MSG_ERR, "Error, RPi not detected\n");
    return -1;
  }

  switch (spi_driver) {
    case DRV_BCM:
      // Map the RPi BCM2835 peripherals - uses "rtapi_open_as_root" in place of "open"
      if (!rt_bcm2835_init()) {
        rtapi_print_msg(RTAPI_MSG_ERR, "rt_bcm2835_init failed. Are you running with root privileges??\n");
        return -1;
      }

      // Set the SPI0 pins to the Alt 0 function to enable SPI0 access, setup CS register
      // and clear TX and RX fifos
      if (!bcm2835_spi_begin()) {
        rtapi_print_msg(RTAPI_MSG_ERR, "bcm2835_spi_begin failed. Are you running with root privlages??\n");
        return -1;
      }

      // Configure SPI0
      bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);  // The default
      bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);               // The default

      // bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_128);
      bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64);
      // bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_32);
      // bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_16);

      bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                  // The default
      bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);  // the default

      /* RPI_GPIO_P1_19        = 10 		MOSI when SPI0 in use
       * RPI_GPIO_P1_21        =  9 		MISO when SPI0 in use
       * RPI_GPIO_P1_23        = 11 		CLK when SPI0 in use
       * RPI_GPIO_P1_24        =  8 		CE0 when SPI0 in use
       * RPI_GPIO_P1_26        =  7 		CE1 when SPI0 in use
       */

      // Configure pullups on SPI0 pins - source termination and CS high (does this allows for higher clock
      // frequencies??? wiring is more important here)
      bcm2835_gpio_set_pud(RPI_GPIO_P1_19, BCM2835_GPIO_PUD_DOWN);  // MOSI
      bcm2835_gpio_set_pud(RPI_GPIO_P1_21, BCM2835_GPIO_PUD_DOWN);  // MISO
      bcm2835_gpio_set_pud(RPI_GPIO_P1_24, BCM2835_GPIO_PUD_UP);    // CS0
      break;
    case DRV_RP1:
      if (!rt_rp1lib_init()) {
        rtapi_print_msg(RTAPI_MSG_ERR, "rt_rp1_init failed.\n");
        return -1;
      }

      if (rp1spi_init(0, 0, SPI_MODE_0, 5000000) != 1)  // SPIx, CSx, mode, freq
      {
        rtapi_print_msg(RTAPI_MSG_ERR, "rp1spi_init failed.\n");
        return -1;
      }
      break;
    default:
      return -1;
  }
  return 0;
}

// This is the same as the standard bcm2835 library except for the use of
// "rtapi_open_as_root" in place of "open"

int rt_bcm2835_init(void) {
  FILE *fp;

  if (debug) {
    bcm2835_peripherals = (uint32_t *)BCM2835_PERI_BASE;

    bcm2835_pads = bcm2835_peripherals + BCM2835_GPIO_PADS / 4;
    bcm2835_clk = bcm2835_peripherals + BCM2835_CLOCK_BASE / 4;
    bcm2835_gpio = bcm2835_peripherals + BCM2835_GPIO_BASE / 4;
    bcm2835_pwm = bcm2835_peripherals + BCM2835_GPIO_PWM / 4;
    bcm2835_spi0 = bcm2835_peripherals + BCM2835_SPI0_BASE / 4;
    bcm2835_bsc0 = bcm2835_peripherals + BCM2835_BSC0_BASE / 4;
    bcm2835_bsc1 = bcm2835_peripherals + BCM2835_BSC1_BASE / 4;
    bcm2835_st = bcm2835_peripherals + BCM2835_ST_BASE / 4;
    bcm2835_aux = bcm2835_peripherals + BCM2835_AUX_BASE / 4;
    bcm2835_spi1 = bcm2835_peripherals + BCM2835_SPI1_BASE / 4;

    return 1; /* Success */
  }

  /* Figure out the base and size of the peripheral address block
  // using the device-tree. Required for RPi2/3/4, optional for RPi 1
  */
  if ((fp = fopen(BMC2835_RPI2_DT_FILENAME, "rb"))) {
    unsigned char buf[16];
    if (fread(buf, 1, sizeof(buf), fp) >= 8) {
      uint32_t base_address = buf[4] << 24 | buf[5] << 16 | buf[6] << 8 | buf[7] << 0;

      uint32_t peri_size = buf[8] << 24 | buf[9] << 16 | buf[10] << 8 | buf[11] << 0;

      if (!base_address) {
        /* looks like RPI 4 */
        base_address = buf[8] << 24 | buf[9] << 16 | buf[10] << 8 | buf[11] << 0;

        peri_size = buf[12] << 24 | buf[13] << 16 | buf[14] << 8 | buf[15] << 0;
      }
      /* check for valid known range formats */
      if (buf[0] == 0x7e && buf[1] == 0x00 && buf[2] == 0x00 && buf[3] == 0x00 &&
          (base_address == BCM2835_PERI_BASE || base_address == BCM2835_RPI2_PERI_BASE ||
           base_address == BCM2835_RPI4_PERI_BASE)) {
        bcm2835_peripherals_base = (off_t)base_address;
        bcm2835_peripherals_size = (size_t)peri_size;
        if (base_address == BCM2835_RPI4_PERI_BASE) {
          pud_type_rpi4 = 1;
        }
      }
    }

    fclose(fp);
  }
  /* Now get ready to map the peripherals block
   * If we are not root, try for the new /dev/gpiomem interface and accept
   * the fact that we can only access GPIO
   * else try for the /dev/mem interface and get access to everything
   */
  int memfd;
  int ok = 0;
  if (geteuid() == 0) {
    /* Open the master /dev/mem device */
    if ((memfd = rtapi_open_as_root("/dev/mem", O_RDWR | O_SYNC)) < 0) {
      fprintf(stderr, "bcm2835_init: Unable to open /dev/mem: %s\n", strerror(errno));
      goto exit;
    }

    /* Base of the peripherals block is mapped to VM */
    bcm2835_peripherals = mapmem("gpio", bcm2835_peripherals_size, memfd, bcm2835_peripherals_base);
    if (bcm2835_peripherals == MAP_FAILED) goto exit;

    /* Now compute the base addresses of various peripherals,
    // which are at fixed offsets within the mapped peripherals block
    // Caution: bcm2835_peripherals is uint32_t*, so divide offsets by 4
    */
    bcm2835_gpio = bcm2835_peripherals + BCM2835_GPIO_BASE / 4;
    bcm2835_pwm = bcm2835_peripherals + BCM2835_GPIO_PWM / 4;
    bcm2835_clk = bcm2835_peripherals + BCM2835_CLOCK_BASE / 4;
    bcm2835_pads = bcm2835_peripherals + BCM2835_GPIO_PADS / 4;
    bcm2835_spi0 = bcm2835_peripherals + BCM2835_SPI0_BASE / 4;
    bcm2835_bsc0 = bcm2835_peripherals + BCM2835_BSC0_BASE / 4; /* I2C */
    bcm2835_bsc1 = bcm2835_peripherals + BCM2835_BSC1_BASE / 4; /* I2C */
    bcm2835_st = bcm2835_peripherals + BCM2835_ST_BASE / 4;
    bcm2835_aux = bcm2835_peripherals + BCM2835_AUX_BASE / 4;
    bcm2835_spi1 = bcm2835_peripherals + BCM2835_SPI1_BASE / 4;

    ok = 1;
  } else {
    /* Not root, try /dev/gpiomem */
    /* Open the master /dev/mem device */
    if ((memfd = open("/dev/gpiomem", O_RDWR | O_SYNC)) < 0) {
      fprintf(stderr, "bcm2835_init: Unable to open /dev/gpiomem: %s\n", strerror(errno));
      goto exit;
    }

    /* Base of the peripherals block is mapped to VM */
    bcm2835_peripherals_base = 0;
    bcm2835_peripherals = mapmem("gpio", bcm2835_peripherals_size, memfd, bcm2835_peripherals_base);
    if (bcm2835_peripherals == MAP_FAILED) goto exit;
    bcm2835_gpio = bcm2835_peripherals;
    ok = 1;
  }

exit:
  if (memfd >= 0) close(memfd);

  if (!ok) bcm2835_close();

  return ok;
}

int rt_rp1lib_init(void) {
  const uint64_t phys_addr = RP1_BAR1;

  DEBUG_PRINT("Initialising RP1 library: %s\n", __func__);

  // rp1_chip is declared in gpiochip_rp1.c
  chip = &rp1_chip;

  inst = rp1_create_instance(chip, phys_addr, NULL);
  if (!inst) return -1;

  inst->phys_addr = phys_addr;

  // map memory
  inst->mem_fd = rtapi_open_as_root("/dev/mem", O_RDWR | O_SYNC);
  if (inst->mem_fd < 0) return errno;

  inst->priv = mmap(NULL, RP1_BAR1_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, inst->mem_fd, (int64_t)inst->phys_addr);

  DEBUG_PRINT("Base address: %11lx, size: %x, mapped at address: %p\n", inst->phys_addr, RP1_BAR1_LEN, inst->priv);

  if (inst->priv == MAP_FAILED) return errno;

  return 1;
}
