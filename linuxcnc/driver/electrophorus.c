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
      hal_float_t *position_fb;   // out: position feedback (position units)
      hal_float_t *velocity_fb;   // out: velocity feedback
      hal_bit_t *enable;          // is the stepper enabled?
      hal_bit_t *position_reset;  // reset position when true
    } pin;

    struct {
      hal_float_t position_scale;  // steps per position unit
      hal_float_t maxaccel;        // max accel (pos units/sec^2)
    } param;
  } hal;

  int64_t prev_step_position;
  int64_t step_position;
} stepper_state_t;

typedef struct {
  stepper_state_t stepgens[STEPGENS];

  hal_bit_t *comms_enable;  // in: are the comms enabled?
  hal_bit_t *comms_reset;   // in: should go low when the machine is pulled out of e-stop
  hal_bit_t *comms_status;  // out: will go low if the comms are not working for some reason

  hal_bit_t *comms_ready;  // out: goes high when UART is opened/initialized

  // HAL param: enable experimental same-tick mode on host side (no behavior change yet)
  hal_bit_t same_tick_mode;

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
  // instrumentation
  hal_s32_t *metric_uart_bytes_avail;
  hal_s32_t *metric_uart_rx_total;

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
// Load-time module parameters
static char *uart = NULL;               // e.g. /dev/ttyAMA0 or /dev/ttyUSB0
RTAPI_MP_STRING(uart, "UART device path (e.g. /dev/ttyAMA0)");
static int baudrate = UART_BAUD;        // e.g. 3000000
RTAPI_MP_INT(baudrate, "UART baudrate (e.g. 3000000)");

static const char *uart_dev_path = UART_DEV;  // set from module param in rtapi_app_main

static speed_t baud_to_constant(int b) {
  switch (b) {
#ifdef B3000000
    case 3000000: return B3000000;
#endif
#ifdef B2000000
    case 2000000: return B2000000;
#endif
#ifdef B1500000
    case 1500000: return B1500000;
#endif
#ifdef B1000000
    case 1000000: return B1000000;
#endif
#ifdef B921600
    case 921600:  return B921600;
#endif
#ifdef B460800
    case 460800:  return B460800;
#endif
    case 230400:  return B230400;
    case 115200:  return B115200;
    default:      return B115200;
  }
}



// Track comms-reset edge and prime/reply handshake across the split read/write
static int last_comms_reset = 0;
static int prime_pending = 0;      // allow first write on reset edge
static int awaiting_reply = 0;     // force next read even while status==0
static int config_pending = 0;     // send PRU_CONF once after reset/open


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
  if (pin_err(hal_param_bit_newf(HAL_RW, &state->same_tick_mode, comp_id, "%s.same-tick-mode", prefix))) return -1;
  state->same_tick_mode = 0;

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

    if (pin_err(hal_param_float_newf(HAL_RW, &par->maxaccel, comp_id, "%s.stepgen.%s.maxaccel", prefix, name)))
      return -1;
    par->maxaccel = 1.0;

    typeof(s->hal.pin) *pin = &s->hal.pin;

    if (pin_err(hal_pin_bit_newf(HAL_IN, &pin->enable, comp_id, "%s.stepgen.%s.enable", prefix, name))) return -1;

    if (pin_err(hal_pin_bit_newf(HAL_IN, &pin->position_reset, comp_id, "%s.stepgen.%s.position-reset", prefix, name)))
      return -1;

    if (pin_err(hal_pin_float_newf(HAL_IN, &pin->position_cmd, comp_id, "%s.stepgen.%s.position-cmd", prefix, name)))
      return -1;
    *pin->position_cmd = 0.0;

    if (pin_err(hal_pin_float_newf(HAL_OUT, &pin->position_fb, comp_id, "%s.stepgen.%s.position-fb", prefix, name)))
      return -1;
    *pin->position_fb = 0.0;

    if (pin_err(hal_pin_float_newf(HAL_OUT, &pin->velocity_fb, comp_id, "%s.stepgen.%s.velocity-fb", prefix, name)))
      return -1;
    *pin->velocity_fb = 0.0;
  }
  // Apply load-time params for UART device and baudrate
  uart_dev_path = (uart && uart[0]) ? uart : UART_DEV;
  rtapi_print("UART: configured device=%s baud=%d\n", uart_dev_path, baudrate);


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
  const int fd = open(uart_dev_path, O_RDWR | O_NOCTTY);
  if (fd < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "failed to open %s: %s\n", uart_dev_path, strerror(errno));
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

  speed_t spd = baud_to_constant(baudrate);
  cfsetispeed(&tio, spd);
  cfsetospeed(&tio, spd);

  if (tcsetattr(fd, TCSANOW, &tio) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "tcsetattr failed: %s\n", strerror(errno));
    close(fd);

    return -1;
  }
  // Try to enable low-latency mode where supported (harmless if unsupported)
#ifdef TIOCGSERIAL
  do {
    struct serial_struct ser;
    if (ioctl(fd, TIOCGSERIAL, &ser) == 0) {
#ifdef ASYNC_LOW_LATENCY
      ser.flags |= ASYNC_LOW_LATENCY;
#endif
      (void)ioctl(fd, TIOCSSERIAL, &ser);
    }
  } while (0);
#endif


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
    config_pending = 0;
    return;
  }


  // Reset rising edge: flush/drain residual and schedule PRU_CONF on first write
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
    prime_pending = 1;            // allow first end-of-tick write
    awaiting_reply = 0;
    config_pending = 1;           // send PRU_CONF at end of this tick
    *state->comms_status = 0;
    // do not return; in same-tick mode we can still do a fresh read now
  }

  if (uart_fd < 0 || !atomic_load_explicit(&uart_healthy_flag, memory_order_acquire)) {
    *state->comms_status = 0;
    return;
  }

  if (state->same_tick_mode) {
    // Same-tick fresh read: send PRU_READ trigger and read reply immediately (guard with poll)
    int32_t read_token = (int32_t)PRU_READ;
    atomic_store_explicit(&uart_io_active, 1, memory_order_release);
    ssize_t w4 = write_exact(uart_fd, (const uint8_t*)&read_token, 4);
    if (w4 < 0) {
      atomic_store_explicit(&uart_io_active, 0, memory_order_release);
      (*state->metric_read_errors)++;
      *state->comms_status = 0;
      return;
    }
    // Wait up to ~1 ms until at least a full frame is buffered to avoid blocking
    int avail = 0;
    int waited_us = 0;
    for (;;) {
      (void)ioctl(uart_fd, FIONREAD, &avail);
      if (state->metric_uart_bytes_avail) *state->metric_uart_bytes_avail = avail;
      if (avail >= (int)XFER_BUF_SIZE) break;
      struct pollfd pfd = { .fd = uart_fd, .events = POLLIN, .revents = 0 };
      (void)poll(&pfd, 1, 0);
      if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL))) {
        atomic_store_explicit(&uart_io_active, 0, memory_order_release);
        (*state->metric_read_errors)++;
        *state->comms_status = 0;
        return;
      }
      if (waited_us >= 1000) {
        atomic_store_explicit(&uart_io_active, 0, memory_order_release);
        (*state->metric_read_errors)++;
        *state->comms_status = 0;
        return;
      }
      struct timespec ts = { .tv_sec = 0, .tv_nsec = 100 * 1000 }; // 100 us
      nanosleep(&ts, NULL);
      waited_us += 100;
    }
    ssize_t rr = read_exact(uart_fd, &rx_frame.buffer[0], XFER_BUF_SIZE);
    atomic_store_explicit(&uart_io_active, 0, memory_order_release);
    if (rr < 0) {
      (*state->metric_read_errors)++;
      *state->comms_status = 0;
      return;
    } else {
      if (state->metric_uart_rx_total) *state->metric_uart_rx_total += (int)rr;
    }
  } else {
    // Pipelined: only read when awaiting reply after a prime or once link is up
    if (!*state->comms_status && !awaiting_reply) return;
    atomic_store_explicit(&uart_io_active, 1, memory_order_release);
    ssize_t rr = read_exact(uart_fd, &rx_frame.buffer[0], XFER_BUF_SIZE);
    atomic_store_explicit(&uart_io_active, 0, memory_order_release);
    if (rr < 0) {
      (*state->metric_read_errors)++;
      *state->comms_status = 0;
      awaiting_reply = 0;
      return;
    }
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
      // velocity_fb from accumulator delta over host servo period
      double steps = (double)acc_delta / (double)FIXED_ONE;
      double vel_steps_s = steps / f_period_s;
      double vel_mu_s = vel_steps_s / s->hal.param.position_scale;
      *s->hal.pin.velocity_fb = (hal_float_t)vel_mu_s;
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

  // If a configuration packet is pending, send it first and skip building PRU_DATA this tick
  if (config_pending) {
    linuxCncConf_t conf = {0};
    conf.header = PRU_CONF;
    for (int i = 0; i < STEPGENS; i++) {
      stepper_state_t *s = &state->stepgens[i];
      conf.stepper_position_scale[i] = (float)s->hal.param.position_scale;
      conf.stepper_max_accel[i] = (float)s->hal.param.maxaccel;
      conf.stepper_init_position[i] = (float)(*s->hal.pin.position_cmd);
    }
    conf.servo_period_s = (float)f_period_s;

    atomic_store_explicit(&uart_io_active, 1, memory_order_release);
    ssize_t ww = write_exact(uart_fd, &conf.buffer[0], XFER_BUF_SIZE);
    atomic_store_explicit(&uart_io_active, 0, memory_order_release);
    if (ww < 0) {
      (*state->metric_write_errors)++;
      // keep config_pending set to retry next tick
      prime_pending = 0;
      awaiting_reply = 0;
    } else {
      config_pending = 0;
      prime_pending = 0;
      awaiting_reply = 1;
    }
    last_comms_reset = *state->comms_reset;
    return;
  }

  linuxCncState_t *txp = &tx_frame;
  txp->header = (state->same_tick_mode ? PRU_WRITE : PRU_DATA);
  txp->stepgen_enable_mask = 0;
  txp->outputs = 0;

  // Pass-through per-axis commanded position, and update enable mask
  for (int i = 0; i < STEPGENS; i++) {
    stepper_state_t *s = &state->stepgens[i];
    txp->stepgen_position_cmd[i] = (float)(*s->hal.pin.position_cmd);
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
      rtapi_print("UART: opening %s\n", uart_dev_path);
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
