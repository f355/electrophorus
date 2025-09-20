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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <pthread.h>
#include <poll.h>
#include <stdatomic.h>

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

  // counters
  hal_s32_t *metric_frames_ok;
  hal_s32_t *metric_bad_header;
  hal_s32_t *metric_read_errors;
  hal_s32_t *metric_write_errors;

  hal_bit_t *inputs[INPUT_PINS * 2];  // digital input pins, twice for inverted 'not' pins
                                      // passed through to LinuxCNC

} state_t;

static state_t *state;

static linuxCncState_t tx_frame;
static pruState_t rx_frame;

static bool pin_err(int retval);

#include "metrics.c"

int comp_id;  // component ID
const char *modname = MODNAME;
const char *prefix = PREFIX;

// UART (FT232R) pipelined exchange support
static void uart_read(void *arg, long l_period_ns);
static void uart_write(void *arg, long l_period_ns);
static int uart_open_config(void);
// UART monitor state
static pthread_t uart_mon_tid;
static _Atomic int mon_run = 0;
static _Atomic int uart_ready_flag = 0;   // 1 when opened and healthy
static _Atomic int uart_healthy_flag = 0; // 1 when poll/ioctl health ok
static _Atomic int uart_io_active = 0;    // 1 while servo read/write is in progress

static void *uart_monitor(void *arg);

static ssize_t read_exact(int fd, uint8_t *p, size_t n);
static ssize_t write_exact(int fd, const uint8_t *p, size_t n);
static int uart_fd = -1;

#define UART_DEV "/dev/ttyUSB0"
#define UART_BAUD 3000000

// Track comms-reset edge and prime/reply handshake across the split read/write
static int last_comms_reset = 0;
static int prime_pending = 0;      // allow first write on reset edge
static int awaiting_reply = 0;     // force next read even while status==0


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
  rtapi_snprintf(name, sizeof(name), "%s.uart-read", prefix);
  int retval = hal_export_funct(name, uart_read, 0, 0, 0, comp_id);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: uart-read export failed\n", modname);
    hal_exit(comp_id);
    return -1;
  }

  rtapi_snprintf(name, sizeof(name), "%s.uart-write", prefix);
  retval = hal_export_funct(name, uart_write, 0, 0, 0, comp_id);
  if (retval < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: uart-write export failed\n", modname);
    hal_exit(comp_id);
    return -1;
  }

  // Spawn UART monitor thread (keeps /dev/ttyUSB0 open and healthy)
  int thr = pthread_create(&uart_mon_tid, NULL, uart_monitor, NULL);
  if (thr != 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pthread_create(uart_monitor) failed: %s\n", modname, strerror(thr));
    hal_exit(comp_id);
    return -1;
  }

  rtapi_print_msg(RTAPI_MSG_INFO, "%s: installed driver\n", modname);
  hal_ready(comp_id);
  return 0;
}

void rtapi_app_exit(void) {
  // Stop monitor thread
  atomic_store_explicit(&mon_run, 0, memory_order_release);
  if (uart_mon_tid) {
    void *ret = NULL;
    pthread_join(uart_mon_tid, &ret);
  }
  if (uart_fd >= 0) {
    close(uart_fd);
    uart_fd = -1;
  }
  hal_exit(comp_id);
}

static bool pin_err(const int retval) {
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


static ssize_t write_exact(const int fd, const uint8_t *p, size_t n) {
  size_t off = 0;
  while (off < n) {
    ssize_t w = write(fd, p + off, n - off);
    if (w < 0) {
      if (errno == EINTR) continue;
      return -1;
    }
    off += (size_t)w;
  }
  return (ssize_t)off;
}

static ssize_t read_exact(const int fd, uint8_t *p, size_t n) {
  size_t off = 0;
  while (off < n) {
    ssize_t r = read(fd, p + off, n - off);
    if (r < 0) {
      if (errno == EINTR) continue;
      return -1;
    }
    off += (size_t)r;
  }
  return (ssize_t)off;
}

static int uart_open_config(void) {
  if (uart_fd >= 0) return 0;
  const int fd = open(UART_DEV, O_RDWR | O_NOCTTY);
  if (fd < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "failed to open %s: %s\n", UART_DEV, strerror(errno));
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
  // Blocking reads: read() blocks until we have a full frame
  tio.c_cc[VMIN] = XFER_BUF_SIZE;  // block until 62 bytes are available
  tio.c_cc[VTIME] = 0;             // no interbyte timeout

  cfsetispeed(&tio, B3000000);
  cfsetospeed(&tio, B3000000);

  if (tcsetattr(fd, TCSANOW, &tio) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "tcsetattr failed: %s\n", strerror(errno));
    close(fd);

    return -1;
  }

  // Flush any stale RX bytes before first use
  tcflush(fd, TCIFLUSH);

  // Ensure blocking mode (clear O_NONBLOCK)
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags >= 0 && (flags & O_NONBLOCK)) fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

  uart_fd = fd;
  return 0;
}

static void uart_read(void *arg, long l_period_ns) {
  (void)arg;
  (void)l_period_ns;

  // Comms disabled or UART not ready: clear status/handshake and skip
  if (!*(state->comms_enable) || !*(state->comms_ready)) {
    *state->comms_status = 0;
    prime_pending = 0;
    awaiting_reply = 0;
    return;
  }


  // Reset rising edge: flush/drain residual; arm first write and skip this read
  const int reset_edge = (*(state->comms_reset) && !last_comms_reset);
  if (reset_edge) {
    if (uart_fd >= 0) {
      tcflush(uart_fd, TCIOFLUSH);
      for (int it = 0; it < 8; ++it) {
        int avail = 0;
        if (ioctl(uart_fd, FIONREAD, &avail) < 0 || avail <= 0) break;
        uint8_t drop[256];
        ssize_t r = read(uart_fd, drop, avail < 256 ? avail : 256);
        if (r <= 0) break;
      }
    }
    prime_pending = 1;
    awaiting_reply = 0;
    *state->comms_status = 0;
    return;
  }

  // If link not yet established, only read when awaiting reply after a prime
  if (!*state->comms_status && !awaiting_reply) return;

  if (uart_fd < 0 || !atomic_load_explicit(&uart_healthy_flag, memory_order_acquire)) {
    *state->comms_status = 0;
    return;
  }

  // Read previous reply (blocking) and apply immediately
  atomic_store_explicit(&uart_io_active, 1, memory_order_release);
  ssize_t rr = read_exact(uart_fd, &rx_frame.buffer[0], XFER_BUF_SIZE);
  atomic_store_explicit(&uart_io_active, 0, memory_order_release);
  if (rr < 0) {
    (*state->metric_read_errors)++;
    *state->comms_status = 0;
    awaiting_reply = 0;
    return;
  }
  const pruState_t *rxp = &rx_frame;
  if (rxp->header == PRU_DATA) {
    *state->comms_status = 1;
    awaiting_reply = 0;
    (*state->metric_frames_ok)++;
    for (int i = 0; i < STEPGENS; i++) {
      stepper_state_t *s = &state->stepgens[i];
      const int64_t acc = rxp->stepgen_feedback[i];
      if (fabs(s->hal.param.position_scale) < 1e-6) {
        if (s->hal.param.position_scale >= 0.0) {
          s->hal.param.position_scale = 1.0;
          rtapi_print_msg(RTAPI_MSG_ERR, "stepgen %d position_scale is too close to 0, resetting to 1.0\n", i);
        } else {
          s->hal.param.position_scale = -1.0;
          rtapi_print_msg(RTAPI_MSG_ERR, "stepgen %d position_scale is too close to 0, resetting to -1.0\n", i);
        }
      }
      int64_t acc_delta = (int64_t)((uint64_t)acc - (uint64_t)s->prev_step_position);
      s->step_position += acc_delta;
      if (*s->hal.pin.position_reset != 0) s->step_position = 0;
      *s->hal.pin.position_fb = (double)s->step_position / FIXED_ONE / s->hal.param.position_scale;
      s->prev_step_position = acc;
    }
    for (int i = 0; i < INPUT_VARS; i++) *state->input_vars[i] = rxp->input_vars[i];
    for (int i = 0; i < INPUT_PINS; i++) {
      if ((rxp->inputs & (1 << i)) != 0) {
        *state->inputs[i] = 1;
        *state->inputs[i + INPUT_PINS] = 0;
      } else {
        *state->inputs[i] = 0;
        *state->inputs[i + INPUT_PINS] = 1;
      }
    }
  } else {
    *state->comms_status = 0;
    awaiting_reply = 0;
    (*state->metric_bad_header)++;
  }
}

static void uart_write(void *arg, long l_period_ns) {
  (void)arg;
  (void)l_period_ns;
  if (!*(state->comms_enable)) return;
  if (!*(state->comms_ready)) return;
  if (uart_fd < 0 || !atomic_load_explicit(&uart_healthy_flag, memory_order_acquire)) return;
  // Servo metrics and stepgen command compute + build TX
  if (*state->metrics_reset) {
    *state->metric_frames_ok = 0;
    *state->metric_bad_header = 0;
    *state->metric_read_errors = 0;
    *state->metric_write_errors = 0;
  }
  metrics_servo_tick(state, l_period_ns);

  linuxCncState_t *txp = &tx_frame;
  txp->header = PRU_DATA;
  txp->stepgen_enable_mask = 0;
  txp->outputs = 0;

  // Compute per-stepper command, update feedback, and enable mask
  for (int i = 0; i < STEPGENS; i++) {
    stepper_state_t *s = &state->stepgens[i];

    const double max_steps_per_s = (double)STEPGEN_FREQUENCY / 2.0;
    const double physical_maxvel = max_steps_per_s / fabs(s->hal.param.position_scale);

    if (s->hal.param.maxvel < 0.0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "stepgen.%02d.maxvel < 0, setting to its absolute value\n", i);
      s->hal.param.maxvel = fabs(s->hal.param.maxvel);
    }
    if (s->hal.param.maxvel > physical_maxvel) {
      rtapi_print_msg(RTAPI_MSG_ERR,
        "stepgen.%02d.maxvel is too big for current step timings (%f steps/s) & position-scale (%f), clipping to max possible (%f)\n",
        i, max_steps_per_s, s->hal.param.position_scale, physical_maxvel);
      s->hal.param.maxvel = physical_maxvel;
    }

    double maxvel = (s->hal.param.maxvel == 0.0) ? physical_maxvel : s->hal.param.maxvel;

    if (s->hal.param.maxaccel < 0.0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "stepgen.%02d.maxaccel < 0, setting to its absolute value\n", i);
      s->hal.param.maxaccel = fabs(s->hal.param.maxaccel);
    }

    double new_vel;
    if (*s->hal.pin.control_type == 0) {
      stepgen_instance_position_control(s, l_period_ns, &new_vel);
    } else {
      new_vel = *s->hal.pin.velocity_cmd;
      if (s->hal.param.maxaccel > 0.0) {
        if (((new_vel - *s->hal.pin.velocity_fb) / f_period_s) > s->hal.param.maxaccel) {
          new_vel = (*s->hal.pin.velocity_fb) + (s->hal.param.maxaccel * f_period_s);
        } else if (((new_vel - *s->hal.pin.velocity_fb) / f_period_s) < -s->hal.param.maxaccel) {
          new_vel = (*s->hal.pin.velocity_fb) - (s->hal.param.maxaccel * f_period_s);
        }
      }
    }

    if (new_vel > maxvel) new_vel = maxvel; else if (new_vel < -maxvel) new_vel = -maxvel;

    *s->hal.pin.velocity_fb = (hal_float_t)new_vel;
    txp->stepgen_freq_command[i] = (int32_t)(new_vel * s->hal.param.position_scale);

    if (*s->hal.pin.enable == 1) txp->stepgen_enable_mask |= 1 << i; else txp->stepgen_enable_mask &= ~(1 << i);
  }

  for (int i = 0; i < OUTPUT_VARS; i++) txp->output_vars[i] = (int32_t)*state->output_vars[i];
  for (int i = 0; i < OUTPUT_PINS; i++) { if (*state->outputs[i] == 1) txp->outputs |= 1 << i; else txp->outputs &= ~(1 << i); }

  int should_write = prime_pending || *state->comms_status;
  if (!should_write) { last_comms_reset = *state->comms_reset; return; }

  atomic_store_explicit(&uart_io_active, 1, memory_order_release);
  ssize_t ww = write_exact(uart_fd, &tx_frame.buffer[0], XFER_BUF_SIZE);
  atomic_store_explicit(&uart_io_active, 0, memory_order_release);
  if (ww < 0) {
    (*state->metric_write_errors)++;
    prime_pending = 0;
    awaiting_reply = 0;
  } else {
    prime_pending = 0;
    awaiting_reply = 1;
  }
  last_comms_reset = *state->comms_reset;
}


static void *uart_monitor(void *arg) {
  (void)arg;
  rtapi_print("UART monitor: start\n");
  atomic_store_explicit(&mon_run, 1, memory_order_release);

  for (;;) {
    if (!atomic_load_explicit(&mon_run, memory_order_acquire)) break;

    if (uart_fd < 0) {
      // Try open
      rtapi_print("UART: opening %s\n", UART_DEV);
      if (uart_open_config() == 0) {
        // Pulse DTR/RTS and flush/drain residual
        int m;
        if (ioctl(uart_fd, TIOCMGET, &m) == 0) {
          m &= ~(TIOCM_DTR | TIOCM_RTS);
          ioctl(uart_fd, TIOCMSET, &m);
          m |= (TIOCM_DTR | TIOCM_RTS);
          ioctl(uart_fd, TIOCMSET, &m);
        }
        tcflush(uart_fd, TCIOFLUSH);
        // Drain any pending bytes
        for (int it = 0; it < 8; ++it) { int avail = 0; if (ioctl(uart_fd, FIONREAD, &avail) < 0 || avail <= 0) break; uint8_t drop[256]; ssize_t r = read(uart_fd, drop, avail < 256 ? avail : 256); if (r <= 0) break; }
        atomic_store_explicit(&uart_healthy_flag, 1, memory_order_release);
        atomic_store_explicit(&uart_ready_flag, 1, memory_order_release);
        if (state && state->comms_ready) *state->comms_ready = 1;
        rtapi_print("UART: open ok fd=%d\n", uart_fd);
      } else {
        atomic_store_explicit(&uart_healthy_flag, 0, memory_order_release);
        atomic_store_explicit(&uart_ready_flag, 0, memory_order_release);
        if (state && state->comms_ready) *state->comms_ready = 0;
        rtapi_print("UART: open failed (errno=%d %s)\n", errno, strerror(errno));
        struct timespec ts = { .tv_sec = 0, .tv_nsec = 500 * 1000 * 1000 }; // 500ms
        nanosleep(&ts, NULL);
        continue;
      }
    } else {
      // Health check
      int healthy = 1;
      int m;
      if (ioctl(uart_fd, TIOCMGET, &m) < 0) healthy = 0;
      struct pollfd p = { .fd = uart_fd, .events = POLLIN | POLLERR | POLLHUP | POLLNVAL, .revents = 0 };
      int pr = poll(&p, 1, 0);
      if (pr >= 0 && (p.revents & (POLLERR | POLLHUP | POLLNVAL))) healthy = 0;

      int prev_ready = atomic_load_explicit(&uart_ready_flag, memory_order_acquire);
      if (!healthy) {
        if (prev_ready) rtapi_print("UART: unhealthy (revents=%#x), reopening\n", p.revents);
        atomic_store_explicit(&uart_healthy_flag, 0, memory_order_release);
        atomic_store_explicit(&uart_ready_flag, 0, memory_order_release);
        if (state && state->comms_ready) *state->comms_ready = 0;
        // Wait for servo I/O to quiesce
        for (int i = 0; i < 50; ++i) { // up to ~50ms
          if (!atomic_load_explicit(&uart_io_active, memory_order_acquire)) break;
          struct timespec ts = { .tv_sec = 0, .tv_nsec = 1 * 1000 * 1000 };
          nanosleep(&ts, NULL);
        }
        if (uart_fd >= 0) { rtapi_print("UART: closing fd=%d\n", uart_fd); close(uart_fd); uart_fd = -1; }
        continue; // next loop will try reopen
      } else {
        if (!prev_ready) { // transitioned to healthy
          atomic_store_explicit(&uart_healthy_flag, 1, memory_order_release);
          atomic_store_explicit(&uart_ready_flag, 1, memory_order_release);
          if (state && state->comms_ready) *state->comms_ready = 1;
          rtapi_print("UART: healthy\n");
        }
      }
    }

    struct timespec ts = { .tv_sec = 0, .tv_nsec = 50 * 1000 * 1000 }; // 50ms
    nanosleep(&ts, NULL);
  }

  rtapi_print("UART monitor: stop\n");
  return NULL;
}
