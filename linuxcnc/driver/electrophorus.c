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

typedef struct {
  // parameters
  hal_float_t scale;     // steps per position unit
  hal_float_t maxvel;    // max velocity, (pos units/sec)
  hal_float_t maxaccel;  // max accel (pos units/sec^2)
  hal_float_t pgain;     // PID proportional gain
  hal_float_t ff1gain;   // PID first derivative gain
  hal_float_t deadband;  // don't try to correct position errors less than this value

  // pins
  hal_bit_t *enabled;     // is the joint enabled?
  hal_float_t *pos_cmd;   // in: position command (position units)
  hal_float_t *pos_fb;    // out: position feedback (position units)
  hal_s32_t *count;       // out: position feedback (raw counts)
  hal_float_t *freq_cmd;  // out: frequency command monitoring, available in LinuxCNC

  // other state
  int32_t prev_count;   // previous count for spike filtering
  int8_t filter_count;  // the number of times spike filter fired
  float freq;           // frequency command sent to PRU
  float old_scale;      // stored scale value
  float prev_cmd;       // previous command to calculate the derivative for PID
} joint_state_t;

typedef struct {
  joint_state_t *joints[JOINTS];

  hal_bit_t *spi_enable;  // in: are SPI comms enabled?
  hal_bit_t *spi_reset;   // in: should go low when the machine is pulled out of e-stop
  hal_bit_t *hard_reset;  // in: will bring the PRU reset pin high. on Carvera Air, the reset pin is the ISP pin,
                          // so this needs to be high for the PRU to boot
  hal_bit_t *spi_status;  // out: will go low if the comms are not working for some reason

  hal_s32_t *output_vars[OUTPUT_VARS];  // output variables: PWM controls, etc.
  hal_s32_t *input_vars[INPUT_VARS];    // input variables: thermistors, pulse counters, etc.
  hal_bit_t *outputs[OUTPUT_PINS];      // digital output pins
  hal_bit_t *inputs[INPUT_PINS * 2];    // digital input pins, twice for inverted 'not' pins
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

enum { BCM = 0, RP1 } spi_driver = BCM;

static long old_dtns;    // update_freq function period in nsec - (THIS IS RUNNING IN THE PI)
static double dt;        // update_freq period in seconds  - (THIS IS RUNNING IN THE PI)
static double recip_dt;  // recprocal of period, avoids divides

static int reset_gpio_pin = 25;  // RPI GPIO pin number used to force watchdog reset of the PRU

static bool pin_err(int retval);
static int rt_peripheral_init();
static int rt_bcm2835_init();
static int rt_rp1lib_init();

static void update_freq(void *arg, long period);
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

  if (rt_peripheral_init() < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "rt_peripheral_init failed.\n");
    return -1;
  }

  switch (spi_driver) {
    case BCM:
      bcm2835_gpio_fsel(reset_gpio_pin, BCM2835_GPIO_FSEL_OUTP);
      break;
    case RP1:
      gpio_set_fsel(reset_gpio_pin, GPIO_FSEL_OUTPUT);
  }

  if (pin_err(hal_pin_bit_newf(HAL_IN, &state->hard_reset, comp_id, "%s.hard-reset", prefix))) return -1;
  if (pin_err(hal_pin_bit_newf(HAL_IN, &state->spi_enable, comp_id, "%s.spi-enable", prefix))) return -1;
  if (pin_err(hal_pin_bit_newf(HAL_IN, &state->spi_reset, comp_id, "%s.spi-reset", prefix))) return -1;
  if (pin_err(hal_pin_bit_newf(HAL_OUT, &state->spi_status, comp_id, "%s.spi-status", prefix))) return -1;

  int n;
  // export all the variables for each joint and pin
  for (n = 0; n < JOINTS; n++) {
    joint_state_t *joint = hal_malloc(sizeof(joint_state_t));
    if (joint == 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc(state->joints[%d]) failed\n", modname, n);
      hal_exit(comp_id);
      return -1;
    }
    state->joints[n] = joint;

    if (pin_err(hal_param_float_newf(HAL_RW, &joint->scale, comp_id, "%s.joint.%01d.scale", prefix, n))) return -1;
    joint->scale = 1.0;
    if (pin_err(hal_param_float_newf(HAL_RW, &joint->maxvel, comp_id, "%s.joint.%01d.maxvel", prefix, n))) return -1;
    joint->maxvel = 0.0;
    if (pin_err(hal_param_float_newf(HAL_RW, &joint->maxaccel, comp_id, "%s.joint.%01d.maxaccel", prefix, n)))
      return -1;
    joint->maxaccel = 1.0;
    if (pin_err(hal_param_float_newf(HAL_RW, &joint->pgain, comp_id, "%s.joint.%01d.pgain", prefix, n))) return -1;
    joint->pgain = 1.0;
    if (pin_err(hal_param_float_newf(HAL_RW, &joint->ff1gain, comp_id, "%s.joint.%01d.ff1gain", prefix, n))) return -1;
    joint->ff1gain = 1.0;
    if (pin_err(hal_param_float_newf(HAL_RW, &joint->deadband, comp_id, "%s.joint.%01d.deadband", prefix, n)))
      return -1;

    if (pin_err(hal_pin_bit_newf(HAL_IN, &joint->enabled, comp_id, "%s.joint.%01d.enable", prefix, n))) return -1;
    if (pin_err(hal_pin_float_newf(HAL_IN, &joint->pos_cmd, comp_id, "%s.joint.%01d.pos-cmd", prefix, n))) return -1;
    *joint->pos_cmd = 0.0;
    if (pin_err(hal_pin_float_newf(HAL_OUT, &joint->freq_cmd, comp_id, "%s.joint.%01d.freq-cmd", prefix, n))) return -1;
    *joint->freq_cmd = 0.0;
    if (pin_err(hal_pin_float_newf(HAL_OUT, &joint->pos_fb, comp_id, "%s.joint.%01d.pos-fb", prefix, n))) return -1;
    *joint->pos_fb = 0.0;
    if (pin_err(hal_pin_s32_newf(HAL_OUT, &joint->count, comp_id, "%s.joint.%01d.counts", prefix, n))) return -1;
    *joint->count = 0;
  }

  for (n = 0; n < OUTPUT_VARS; n++) {
    const char *output_var_names[OUTPUT_VARS] = OUTPUT_VAR_NAMES;
    if (pin_err(
            hal_pin_s32_newf(HAL_IN, &state->output_vars[n], comp_id, "%s.output-var.%s", prefix, output_var_names[n])))
      return -1;
    *state->output_vars[n] = 0;
  }

  for (n = 0; n < INPUT_VARS; n++) {
    const char *input_var_names[INPUT_VARS] = INPUT_VAR_NAMES;
    if (pin_err(
            hal_pin_s32_newf(HAL_OUT, &state->input_vars[n], comp_id, "%s.input-var.%s", prefix, input_var_names[n])))
      return -1;
    *state->input_vars[n] = 0;
  }

  for (n = 0; n < OUTPUT_PINS; n++) {
    const outputPin_t output_pins[OUTPUT_PINS] = OUTPUT_PIN_DESC;
    if (pin_err(hal_pin_bit_newf(HAL_IN, &state->outputs[n], comp_id, "%s.output.%s", prefix, output_pins[n].name)))
      return -1;
    *state->outputs[n] = 0;
  }

  for (n = 0; n < INPUT_PINS; n++) {
    const inputPin_t input_pins[INPUT_PINS] = INPUT_PIN_DESC;
    if (pin_err(hal_pin_bit_newf(HAL_OUT, &state->inputs[n], comp_id, "%s.input.%s", prefix, input_pins[n].name)))
      return -1;
    *state->inputs[n] = 0;

    // inverted 'not' pins offset by the number of inputs we have.
    if (pin_err(hal_pin_bit_newf(HAL_OUT, &state->inputs[n + INPUT_PINS], comp_id, "%s.input.%s.not", prefix,
                                 input_pins[n].name)))
      return -1;
    *state->inputs[n + INPUT_PINS] = 0;
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

// TODO(f355): do we need all this? LinuxCNC's standard PID component would probably do a better job.
void update_freq(void *arg, const long period) {
  const state_t *state = arg;

  // calc constants related to the period of this function (LinuxCNC SERVO_THREAD)
  // only recalc constants if period changes
  if (period != old_dtns) {     // Note!! period = LinuxCNC SERVO_PERIOD
    old_dtns = period;          // get ready to detect future period changes
    recip_dt = 1.0e9 / period;  // calc the reciprocal once here, to avoid multiple divides later
    dt = 1.0 / recip_dt;        // dt is the period of this thread, used for the position loop
  }

  // loop through generators
  for (int i = 0; i < JOINTS; i++) {
    joint_state_t *joint = state->joints[i];
    // check for scale change
    if (joint->scale != joint->old_scale) {
      joint->old_scale = joint->scale;  // get ready to detect future scale changes
      // scale must not be 0
      if ((joint->scale < 1e-20) && (joint->scale > -1e-20))  // validate the new scale value
        joint->scale = 1.0;                                   // value too small, divide by zero is a bad thing
    }

    // calculate frequency limit
    double max_freq = BASE_THREAD_FREQUENCY;

    // check for user specified frequency limit parameter
    if (joint->maxvel <= 0.0) {
      // set to zero if negative
      joint->maxvel = 0.0;
    } else {
      // parameter is non-zero, compare to max_freq
      const double desired_freq = joint->maxvel * fabs(joint->scale);

      if (desired_freq > max_freq) {
        // parameter is too high, limit it
        joint->maxvel = max_freq / fabs(joint->scale);
      } else {
        // lower max_freq to match parameter
        max_freq = joint->maxvel * fabs(joint->scale);
      }
    }

    /* set internal accel limit to its absolute max, which is
    zero to full speed in one thread period */
    double max_ac = max_freq * recip_dt;

    // check for user specified accel limit parameter
    if (joint->maxaccel <= 0.0) {
      // set to zero if negative
      joint->maxaccel = 0.0;
    } else {
      // parameter is non-zero, compare to max_ac
      if (joint->maxaccel * fabs(joint->scale) > max_ac) {
        // parameter is too high, lower it
        joint->maxaccel = max_ac / fabs(joint->scale);
      } else {
        // lower limit to match parameter
        max_ac = joint->maxaccel * fabs(joint->scale);
      }
    }

    /* at this point, all scaling, limits, and other parameter
    changes have been handled - time for the main control */

    double vel_cmd = 0.0;

    const double command = *joint->pos_cmd;
    const double feedback = *joint->pos_fb;
    const double error = command - feedback;

    double deadband;

    if (joint->deadband != 0) {
      deadband = joint->deadband;
    } else {
      // default deadband to a single step length
      deadband = 1 / joint->scale;
    }

    // use Proportional control with feed forward (pgain, ff1gain and deadband)
    if (fabs(error) > fabs(deadband)) {
      const float cmd_d = (command - joint->prev_cmd) * recip_dt;
      joint->prev_cmd = command;
      vel_cmd = joint->pgain * error + cmd_d * joint->ff1gain;
    }

    vel_cmd *= joint->scale;

    // apply frequency limit
    if (vel_cmd > max_freq) {
      vel_cmd = max_freq;
    } else if (vel_cmd < -max_freq) {
      vel_cmd = -max_freq;
    }

    // calc max change in frequency in one period
    const double dv = max_ac * dt;
    double new_vel;
    // apply accel limit
    if (vel_cmd > joint->freq + dv) {
      new_vel = joint->freq + dv;
    } else if (vel_cmd < joint->freq - dv) {
      new_vel = joint->freq - dv;
    } else {
      new_vel = vel_cmd;
    }

    // test for disabled stepgen
    if (*joint->enabled == 0) {
      // set velocity to zero
      new_vel = 0;
    }

    joint->freq = new_vel;           // to be sent to the PRU
    *joint->freq_cmd = joint->freq;  // feedback to LinuxCNC
  }
}

void spi_read() {
  int i;

  // Data header
  tx_data.header = PRU_READ;

  // update the PRUreset output
  if (*(state->hard_reset)) {
    switch (spi_driver) {
      case BCM:
        bcm2835_gpio_set(reset_gpio_pin);
        break;
      case RP1:
        gpio_set(reset_gpio_pin);
    }
  } else {
    switch (spi_driver) {
      case BCM:
        bcm2835_gpio_clr(reset_gpio_pin);
        break;
      case RP1:
        gpio_clear(reset_gpio_pin);
    }
  }

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

          for (i = 0; i < JOINTS; i++) {
            joint_state_t *joint = state->joints[i];

            // TODO(f355): do we need spike filtering? how could those spikes happen with closed-loop motors?
            // at the moment, it just tries to filter non-zeroes when LinuxCNC is restarted with the PRU running,
            // which makes no sense at all.
            // Feedback spike filter parameters
            const int M = 250;
            const int n = 2;
            int32_t current_count = rx_data.joint_feedback[i];
            const int32_t accum_diff = current_count - joint->prev_count;

            // spike filter
            if (abs(accum_diff) > M && joint->filter_count < n) {
              // recent big change: hold previous value
              joint->filter_count++;
              current_count = joint->prev_count;
              rtapi_print("Spike filter active[%d][%d]: %d\n", i, joint->filter_count, accum_diff);
            } else {
              // normal operation, or else the big change must be real after all
              joint->prev_count = current_count;
              joint->filter_count = 0;
            }

            *joint->count = current_count;
            *joint->pos_fb = (float)current_count / joint->scale;
          }

          for (i = 0; i < INPUT_VARS; i++) {
            *state->input_vars[i] = rx_data.input_vars[i];
          }

          // Inputs
          for (i = 0; i < INPUT_PINS; i++) {
            if ((rx_data.inputs & (1 << i)) != 0) {
              *state->inputs[i] = 1;               // input is high
              *state->inputs[i + INPUT_PINS] = 0;  // inverted 'not' is offset by number of digital inputs.
            } else {
              *state->inputs[i] = 0;               // input is low
              *state->inputs[i + INPUT_PINS] = 1;  // inverted 'not' is offset by number of digital inputs.
            }
          }
          break;

        case PRU_ESTOP:
          // we have an eStop notification from the PRU
          *state->spi_status = 0;
          rtapi_print_msg(RTAPI_MSG_ERR, "An E-stop is active");
          break;

        default:
          // we have received a BAD payload from the PRU
          *state->spi_status = 0;
          rtapi_print("Bad SPI payload = %x\n", rx_data.header);
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

  for (i = 0; i < JOINTS; i++) {
    joint_state_t *joint = state->joints[i];
    tx_data.joint_freq_command[i] = joint->freq;
    if (*joint->enabled == 1) {
      tx_data.joint_enable |= 1 << i;
    } else {
      tx_data.joint_enable &= ~(1 << i);
    }
  }

  for (i = 0; i < OUTPUT_VARS; i++) {
    tx_data.output_vars[i] = *state->output_vars[i];
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
    case BCM:
      // TODO(f355): why transfer byte by byte?
      // bcm2835_spi_transfernb(tx_data.buffer, rx_data.buffer, SPI_BUF_SIZE);
      for (int i = 0; i < SPI_BUF_SIZE; i++) {
        rx_data.buffer[i] = bcm2835_spi_transfer(tx_data.buffer[i]);
      }
      break;
    case RP1:
      rp1spi_transfer(0, tx_data.buffer, rx_data.buffer, SPI_BUF_SIZE);
  }
}

int rt_peripheral_init(void) {
  FILE *fp;

  // assume we're only running on >RPi3

  if ((fp = fopen("/proc/device-tree/soc/ranges", "rb"))) {
    unsigned char buf[16];
    uint32_t base_address;
    if (fread(buf, 1, sizeof(buf), fp) >= 8) {
      base_address = buf[4] << 24 | buf[5] << 16 | buf[6] << 8 | buf[7] << 0;

      if (!base_address) {
        /* looks like RPI 4 or 5 */
        base_address = buf[8] << 24 | buf[9] << 16 | buf[10] << 8 | buf[11] << 0;
      }
    } else {
      rtapi_print_msg(RTAPI_MSG_ERR, "Error reading /proc/device-tree/soc/ranges, not running on a Raspberry Pi?\n");
      return -1;
    }

    switch (base_address) {
      case BCM2835_RPI2_PERI_BASE:
        rtapi_print_msg(RTAPI_MSG_INFO, "Raspberry Pi 3, using BCM2835 driver\n\n");
        spi_driver = BCM;
        break;
      case BCM2835_RPI4_PERI_BASE:
        rtapi_print_msg(RTAPI_MSG_INFO, "Raspberry Pi 4, using BCM2835 driver\n\n");
        spi_driver = BCM;
        break;
      case RPI5_RP1_PERI_BASE:
        rtapi_print_msg(RTAPI_MSG_INFO, "Raspberry Pi 5, using RP1 driver\n\n");
        spi_driver = RP1;
        break;
      default:
        rtapi_print_msg(RTAPI_MSG_ERR, "Error, RPi not detected\n");
        return -1;
    }

    fclose(fp);
  }

  switch (spi_driver) {
    case BCM:
      // Map the RPi BCM2835 peripherals - uses "rtapi_open_as_root" in place of "open"
      if (!rt_bcm2835_init()) {
        rtapi_print_msg(RTAPI_MSG_ERR, "rt_bcm2835_init failed. Are you running with root privlages??\n");
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
    case RP1:
      if (!rt_rp1lib_init()) {
        rtapi_print_msg(RTAPI_MSG_ERR, "rt_rp1_init failed.\n");
        return -1;
      }

      // TODO: Allow user to select SPI number, CS number and frequency// SPIx, CSx, mode, freq. Clock frequency here is
      // different than Pi4, this will get rounded to nearest clock divider. TODO Figure out exact value that works
      // best.
      rp1spi_init(0, 0, SPI_MODE_0, 40000000);
  }
  return 1;
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

  inst->priv = mmap(NULL, RP1_BAR1_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, inst->mem_fd, inst->phys_addr);

  DEBUG_PRINT("Base address: %11lx, size: %x, mapped at address: %p\n", inst->phys_addr, RP1_BAR1_LEN, inst->priv);

  if (inst->priv == MAP_FAILED) return errno;

  return 1;
}
