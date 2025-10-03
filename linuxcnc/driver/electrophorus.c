/**
 * Description:  electrophorus.c
 *               A HAL component that provides a USB-to-serial connection to a Carvera-family machine
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

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <linux/serial.h>

#include "hal.h"
#include "protocol_definitions.h"
#include "rtapi.h"
#include "rtapi_app.h"

#define MODNAME "electrophorus"
#define PREFIX "carvera"

MODULE_AUTHOR("Scott Alford AKA scotta, modified by Konstantin Tcepliaev <f355@f355.org>");
MODULE_DESCRIPTION("Driver for the Carvera family of desktop milling machines");
MODULE_LICENSE("GPL v3");

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

  int64_t prev_step_position;
  int64_t step_position;
} stepper_state_t;

typedef struct {
  stepper_state_t stepgens[STEPGENS];

  hal_bit_t *comms_enable;  // in: are the comms enabled?
  hal_bit_t *comms_reset;   // in: should go low when the machine is pulled out of e-stop
  hal_bit_t *comms_status;  // out: will go low if the comms are not working for some reason

  hal_bit_t *comms_ready;  // out: goes high when UART is opened/initialized

  hal_float_t *output_vars[OUTPUT_VARS];  // output variables: PWM controls, etc.
  hal_float_t *input_vars[INPUT_VARS];    // input variables: thermistors, pulse counters, etc.
  hal_bit_t *outputs[OUTPUT_PINS];        // digital output pins
  // metrics
  hal_bit_t *metrics_reset;  // in: pulse to reset metrics

  // servo timing stats (us) — 1s window
  hal_s32_t *servo_us_1s_min;
  hal_s32_t *servo_us_1s_max;
  hal_s32_t *servo_us_1s_mean;
  hal_s32_t *servo_us_1s_stddev;
  hal_s32_t *servo_us_1s_p50;
  hal_s32_t *servo_us_1s_p95;
  hal_s32_t *servo_us_1s_p99;
  hal_s32_t *servo_us_1s_p99_9;
  // servo timing stats (us) — 1m window
  hal_s32_t *servo_us_1m_mean;
  hal_s32_t *servo_us_1m_stddev;
  hal_s32_t *servo_us_1m_p50;
  hal_s32_t *servo_us_1m_p95;
  hal_s32_t *servo_us_1m_p99;
  hal_s32_t *servo_us_1m_p99_9;

  // UART RTT stats (us) — 1s window
  hal_s32_t *rtt_us_1s_min;
  hal_s32_t *rtt_us_1s_max;
  hal_s32_t *rtt_us_1s_mean;
  hal_s32_t *rtt_us_1s_stddev;
  hal_s32_t *rtt_us_1s_p50;
  hal_s32_t *rtt_us_1s_p95;
  hal_s32_t *rtt_us_1s_p99;
  hal_s32_t *rtt_us_1s_p99_9;
  // UART RTT stats (us) — 1m window
  hal_s32_t *rtt_us_1m_mean;
  hal_s32_t *rtt_us_1m_stddev;
  hal_s32_t *rtt_us_1m_p50;
  hal_s32_t *rtt_us_1m_p95;
  hal_s32_t *rtt_us_1m_p99;
  hal_s32_t *rtt_us_1m_p99_9;

  // Frame age (us) — 1m window
  hal_s32_t *age_us_1m_p50;
  hal_s32_t *age_us_1m_p95;
  hal_s32_t *age_us_1m_p99;

  // counters
  hal_s32_t *metric_frames_ok;
  hal_s32_t *metric_bad_header;
  hal_s32_t *metric_read_errors;
  hal_s32_t *metric_write_errors;

  // RX instrumentation (instantaneous, per tick)
  hal_s32_t *rx_inq_before;
  hal_s32_t *rx_inq_after;
  hal_s32_t *rx_bytes_read;
  hal_s32_t *rx_tail_after_last;
  hal_s32_t *age_us_last;
  hal_s32_t *age_ticks_last;

  // TX instrumentation
  hal_s32_t *tx_outq_after;
  hal_s32_t *tx_bytes;

  hal_bit_t *inputs[INPUT_PINS * 2];  // digital input pins, twice for inverted 'not' pins
                                      // passed through to LinuxCNC

} state_t;

static state_t *state;

static linuxCncState_t linuxcnc_state;
static pruState_t pru_state;

static bool pin_err(int retval);

#include "metrics.c"

int comp_id;  // component ID
// Last TX timestamp we stamped (host side), for correlation
static uint16_t last_tx_ts16 = 0;
static uint16_t prev_tx_ts16 = 0;

const char *modname = MODNAME;
const char *prefix = PREFIX;

// UART (FT232R) pipelined exchange support
static void uart_read();
static void uart_write();
static int uart_open_config(void);
static ssize_t read_exact(int fd, uint8_t *p, size_t n);
static ssize_t write_exact(int fd, const uint8_t *p, size_t n);
static int uart_fd = -1;
// Large RX scratch buffer to avoid big stack frames in uart_read
static uint8_t uart_rx_buf[4096];

static void *uart_init_worker(void *arg);

static char *device = "/dev/ttyUSB0";  // e.g. /dev/ttyUSB0 or /dev/ttyAMA0
#define UART_BAUD 3000000
RTAPI_MP_STRING(device, "Serial device path for comms (e.g., /dev/ttyUSB0 or /dev/ttyAMA0)");

// Non-RT init worker controls
static pthread_t init_thread;
static pthread_mutex_t init_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t init_cv = PTHREAD_COND_INITIALIZER;
static volatile int init_stop = 0;
// init_request: 0=none, 1=open, 2=close
static volatile int init_request = 0;
// init_ready: 1 when UART is opened+initialized by worker
static volatile int init_ready = 0;

// Track comms-enable edge for clean start (reset USB-serial side)
static int last_comms_enable = 0;

static void update_freq(void *arg, long l_period_ns);

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

  if (pin_err(hal_pin_bit_newf(HAL_IN, &state->comms_enable, comp_id, "%s.comms-enable", prefix))) return -1;
  if (pin_err(hal_pin_bit_newf(HAL_IN, &state->comms_reset, comp_id, "%s.comms-reset", prefix))) return -1;
  if (pin_err(hal_pin_bit_newf(HAL_OUT, &state->comms_status, comp_id, "%s.comms-status", prefix))) return -1;
  if (metrics_export_pins(comp_id, prefix, state) < 0) return -1;
  if (pin_err(hal_pin_bit_newf(HAL_OUT, &state->comms_ready, comp_id, "%s.comms-ready", prefix))) return -1;

  *state->comms_ready = 0;

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

  for (int i = 0; i < STEPGENS; i++) {
    state->stepgens[i].prev_step_position = 0;
    state->stepgens[i].step_position = 0;
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

  rtapi_snprintf(name, sizeof(name), "%s.uart-read", prefix);
  retval = hal_export_funct(name, uart_read, 0, 0, 0, comp_id);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: uart-read function export failed\n", modname);
    hal_exit(comp_id);
    return -1;
  }

  rtapi_snprintf(name, sizeof(name), "%s.uart-write", prefix);
  retval = hal_export_funct(name, uart_write, 0, 0, 0, comp_id);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: uart-write function export failed\n", modname);
    hal_exit(comp_id);
    return -1;
  }

  // Start non-RT init worker (explicit SCHED_OTHER, no affinity)
  {
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_OTHER);
    const int err = pthread_create(&init_thread, &attr, uart_init_worker, NULL);
    pthread_attr_destroy(&attr);
    if (err != 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pthread_create failed: %d\n", modname, err);
    }
  }

  rtapi_print_msg(RTAPI_MSG_INFO, "%s: installed driver\n", modname);
  hal_ready(comp_id);
  return 0;
}

void rtapi_app_exit(void) {
  // Stop non-RT worker and clean up
  pthread_mutex_lock(&init_mtx);
  init_stop = 1;
  pthread_cond_broadcast(&init_cv);
  pthread_mutex_unlock(&init_mtx);
  pthread_join(init_thread, NULL);

  if (uart_fd >= 0) {
    close(uart_fd);
    uart_fd = -1;
  }
  hal_exit(comp_id);
}

bool pin_err(const int retval) {
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin export failed with err=%i\n", modname, retval);
    hal_exit(comp_id);
    return true;
  }
  return false;
}

static void stepgen_instance_position_control(stepper_state_t *s, const long l_period_ns, double *new_vel) {
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

  // metrics: sliding servo timing window
  if (*state->metrics_reset) {
    *state->metric_frames_ok = 0;
    *state->metric_bad_header = 0;
    *state->metric_read_errors = 0;
    *state->metric_write_errors = 0;
  }
  metrics_servo_tick(state, l_period_ns);

  // loop through generators
  for (int i = 0; i < STEPGENS; i++) {
    stepper_state_t *s = &state->stepgens[i];

    // first sanity-check our maxaccel and maxvel params
    // maxvel must be >= 0.0, and may not be faster than 1 step per (steplen+stepspace) seconds

    const double max_steps_per_s = (double)STEPGEN_FREQUENCY / 2.0;

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

    linuxcnc_state.stepgen_freq_command[i] = new_vel * s->hal.param.position_scale;
  }
}

static int uart_open_config(void) {
  if (uart_fd >= 0) return 0;
  const int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "failed to open %s: %s\n", device, strerror(errno));
    return -1;
  }

  struct termios tio;
  if (tcgetattr(fd, &tio) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "tcgetattr failed: %s\n", strerror(errno));
    close(fd);
    return -1;
  }

  cfmakeraw(&tio);
  tio.c_cflag |= (CLOCAL | CREAD | CS8);
  tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);
  // Non-blocking reads: drain whatever is available each servo tick
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;

  cfsetispeed(&tio, B3000000);
  cfsetospeed(&tio, B3000000);

  if (tcsetattr(fd, TCSANOW, &tio) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "tcsetattr failed: %s\n", strerror(errno));
    close(fd);

    return -1;
  }

  // Flush any stale RX bytes before first use
  tcflush(fd, TCIFLUSH);

  // Ensure non-blocking mode
  const int flags = fcntl(fd, F_GETFL, 0);
  // Request low-latency delivery from the serial driver if supported
  struct serial_struct ser = {0};
  if (ioctl(fd, TIOCGSERIAL, &ser) == 0) {
    ser.flags |= ASYNC_LOW_LATENCY;
    (void)ioctl(fd, TIOCSSERIAL, &ser);
  }

  if (flags >= 0 && !(flags & O_NONBLOCK)) fcntl(fd, F_SETFL, flags | O_NONBLOCK);

  uart_fd = fd;
  return 0;
}

// CRC32 IEEE 802.3 over len bytes
static uint32_t crc32_ieee(const uint8_t *data, const size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      const uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

static void uart_read() {
  if (!*state->comms_enable) {
    *state->comms_ready = 0;
    *state->comms_status = 0;
    if (last_comms_enable) {
      pthread_mutex_lock(&init_mtx);
      init_request = 2;  // CLOSE
      pthread_cond_signal(&init_cv);
      pthread_mutex_unlock(&init_mtx);
    }
    last_comms_enable = 0;
    return;
  }

  const int rising_edge = !last_comms_enable;
  last_comms_enable = 1;

  if (rising_edge) {
    // Request open+init in non-RT worker and skip I/O this tick
    pthread_mutex_lock(&init_mtx);
    init_ready = 0;
    init_request = 1;  // OPEN
    pthread_cond_signal(&init_cv);
    pthread_mutex_unlock(&init_mtx);
    *state->comms_ready = 0;
    *state->comms_status = 0;
    return;
  }

  if (uart_fd < 0 || !init_ready) {
    *state->comms_ready = 0;
    *state->comms_status = 0;
    return;
  }

  *state->comms_ready = 1;

  // RX: drain available bytes, bounded linger to catch just-arrived data; pick last good frame
  int n = 0;
  struct timespec rd_start; clock_gettime(CLOCK_MONOTONIC_RAW, &rd_start);
  // snapshot bytes queued in driver before draining
  int inq_before = 0; (void)ioctl(uart_fd, FIONREAD, &inq_before);
  // initial nonblocking drain
  for (;;) {
    const ssize_t r = read(uart_fd, uart_rx_buf + n, (int)sizeof(uart_rx_buf) - n);
    if (r > 0) {
      n += (int)r;
      if (n >= (int)sizeof(uart_rx_buf)) break;
      continue;
    }
    if (r < 0 && (errno == EINTR)) continue;
    if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
    break;  // r == 0 or other error
  }
  *state->rx_inq_before = inq_before;
  *state->rx_bytes_read = n;

  int have_frame = 0;
  int last_i = -1;
  for (int i = n - (int)PRU_TO_HOST_FRAME_BYTES; i >= 0; --i) {
    const pruState_t *f = (const pruState_t *)(uart_rx_buf + i);
    if (f->header != PRU_DATA) continue;
    const uint32_t calc = crc32_ieee(((const uint8_t *)f) + 4, sizeof(pruState_t) - 8);
    if (calc == f->crc) {
      memcpy(&pru_state, f, sizeof(pruState_t));
      have_frame = 1;  // found last valid frame
      last_i = i;
      break;
    }
  }

  if (!have_frame) {
    *state->comms_status = 0;  // immediate fault on miss this tick
    (*state->metric_read_errors)++;
    int inq_after2 = 0; (void)ioctl(uart_fd, FIONREAD, &inq_after2);
    *state->rx_inq_after = inq_after2;
    *state->rx_tail_after_last = n;  // no frame found; report total bytes
    return;
  }

  // tail bytes after the end of the newest valid frame we found
  int tail = n - (last_i + (int)PRU_TO_HOST_FRAME_BYTES);
  if (tail < 0) tail = 0;
  *state->rx_tail_after_last = tail;
  int inq_after = 0; (void)ioctl(uart_fd, FIONREAD, &inq_after);
  *state->rx_inq_after = inq_after;

  // Debug: every 300th tick, print diffs of echoed timestamps for all full frames drained this tick
  {
    static int dbg_tick = 0;
    dbg_tick++;
    if (dbg_tick % 300 == 0) {
      uint16_t ts_list[16];
      int ts_count = 0;
      for (int i = 0; i <= n - (int)PRU_TO_HOST_FRAME_BYTES && ts_count < 16; ++i) {
        const pruState_t *ff = (const pruState_t *)(uart_rx_buf + i);
        if (ff->header != PRU_DATA) continue;
        const uint32_t c2 = crc32_ieee(((const uint8_t *)ff) + 4, sizeof(pruState_t) - 8);
        if (c2 == ff->crc) {
          ts_list[ts_count++] = ff->timestamp;
          i += (int)PRU_TO_HOST_FRAME_BYTES - 1; // skip past this frame
        }
      }
      if (ts_count >= 2) {
        char line[256];
        int pos = 0;
        pos += snprintf(line, sizeof(line), "carvera: ts diffs (%d):", ts_count);
        int max_print = ts_count - 1;
        if (max_print > 8) max_print = 8; // bound line length
        for (int k = 1; k <= max_print; ++k) {
          uint16_t d = (uint16_t)(ts_list[k] - ts_list[k - 1]);
          pos += snprintf(line + pos, sizeof(line) - (size_t)pos, " %u", (unsigned)d);
          if (pos >= (int)sizeof(line)) break;
        }
        rtapi_print("%s\n", line);
      }
    }
  }

  // Compute packet age (us) and estimate tick lag
  {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)((ts.tv_nsec + 500ull) / 1000ull);
    uint16_t now16 = (uint16_t)(now_us & 0xFFFFu);
    uint16_t ts16 = pru_state.timestamp;
    int32_t age_us = (int32_t)((uint16_t)(now16 - ts16));  // modulo-16-bit difference
    *state->age_us_last = age_us;
    int32_t lag = 0;
    if (servo_center_us > 0) {
      int32_t per = (int32_t)servo_center_us;
      // round to nearest tick
      lag = (age_us + (per / 2)) / per;
      if (lag < 0) lag = 0; if (lag > 10) lag = 10;
    }
    *state->age_ticks_last = lag;
    metrics_age_sample(state, age_us);
  }

  *state->comms_status = 1;
  (*state->metric_frames_ok)++;

  for (int i = 0; i < STEPGENS; i++) {
    stepper_state_t *s = &state->stepgens[i];
    const int64_t acc = pru_state.stepgen_feedback[i];

    if (fabs(s->hal.param.position_scale) < 1e-6) {
      if (s->hal.param.position_scale >= 0.0) {
        s->hal.param.position_scale = 1.0;
        rtapi_print_msg(RTAPI_MSG_ERR, "stepgen %d position_scale is too close to 0, resetting to 1.0\n", i);
      } else {
        s->hal.param.position_scale = -1.0;
        rtapi_print_msg(RTAPI_MSG_ERR, "stepgen %d position_scale is too close to 0, resetting to -1.0\n", i);
      }
    }

    const int64_t acc_delta = (int64_t)((uint64_t)acc - (uint64_t)s->prev_step_position);
    s->step_position += acc_delta;
    if (*s->hal.pin.position_reset != 0) s->step_position = 0;
    *s->hal.pin.position_fb = (double)s->step_position / FIXED_ONE / s->hal.param.position_scale;
    s->prev_step_position = acc;
  }

  for (int i = 0; i < INPUT_VARS; i++) *state->input_vars[i] = pru_state.input_vars[i];
  for (int i = 0; i < INPUT_PINS; i++) {
    if ((pru_state.inputs & (1 << i)) != 0) {
      *state->inputs[i] = 1;
      *state->inputs[i + INPUT_PINS] = 0;
    } else {
      *state->inputs[i] = 0;
      *state->inputs[i + INPUT_PINS] = 1;
    }
  }
}

static void uart_write() {
  if (!*(state->comms_enable) || uart_fd < 0 || !init_ready) return;

  // Update enable mask and outputs each tick from HAL pins
  for (int i = 0; i < STEPGENS; i++) {
    const stepper_state_t *s = &state->stepgens[i];
    if (*s->hal.pin.enable == 1) {
      linuxcnc_state.stepgen_enable_mask |= 1 << i;
    } else {
      linuxcnc_state.stepgen_enable_mask &= ~(1 << i);
    }
  }
  for (int i = 0; i < OUTPUT_VARS; i++) linuxcnc_state.output_vars[i] = (int32_t)*state->output_vars[i];
  for (int i = 0; i < OUTPUT_PINS; i++) {
    if (*state->outputs[i] == 1)
      linuxcnc_state.outputs |= 1 << i;
    else
      linuxcnc_state.outputs &= ~(1 << i);
  }

  // TX: build and write one frame; attempt to queue the whole frame this tick
  linuxCncState_t tx = linuxcnc_state;
  tx.header = PRU_WRITE;
  // Fill 16-bit timestamp in microseconds (mod 65536) from CLOCK_MONOTONIC_RAW
  {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)((ts.tv_nsec + 500ull) / 1000ull);
    prev_tx_ts16 = last_tx_ts16;
    last_tx_ts16 = (uint16_t)(now_us & 0xFFFFu);
    tx.timestamp = last_tx_ts16;
  }
  tx.crc = crc32_ieee(((const uint8_t *)&tx) + 4, sizeof(linuxCncState_t) - 8);

  const uint8_t *p = (const uint8_t *)&tx;
  ssize_t remain = (ssize_t)sizeof(tx);
  ssize_t wrote = 0;
  while (remain > 0) {
    ssize_t w = write(uart_fd, p, (size_t)remain);
    if (w > 0) { p += w; remain -= w; wrote += w; continue; }
    if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      (*state->metric_write_errors)++;  // couldn't finish this tick
      break;
    }
    if (w < 0) { (*state->metric_write_errors)++; break; }
  }
  *state->tx_bytes = (int32_t)wrote;
  int outq = 0; (void)ioctl(uart_fd, TIOCOUTQ, &outq);
  *state->tx_outq_after = outq;
}

// Non-RT worker: handles UART open/init and close outside the servo thread
static void *uart_init_worker(void *arg) {
  (void)arg;
  // Keep worker thread off the RT core (pin to CPU 1)
  cpu_set_t set;
  CPU_ZERO(&set);
  int worker_cpu = 1;
  CPU_SET(worker_cpu, &set);
  (void)pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
  for (;;) {
    pthread_mutex_lock(&init_mtx);
    while (!init_stop && init_request == 0) {
      pthread_cond_wait(&init_cv, &init_mtx);
    }
    if (init_stop) {
      pthread_mutex_unlock(&init_mtx);
      break;
    }
    const int req = init_request;
    init_request = 0;
    pthread_mutex_unlock(&init_mtx);

    if (req == 2) {  // close
      if (uart_fd >= 0) {
        close(uart_fd);
        uart_fd = -1;
      }
      init_ready = 0;
      continue;
    }

    if (req == 1) {  // open+init
      init_ready = 0;
      if (uart_fd >= 0) {
        close(uart_fd);
        uart_fd = -1;
      }
      if (uart_open_config() < 0) {
        init_ready = 0;
        continue;
      }
      init_ready = 1;
    }
  }
  return NULL;
}
