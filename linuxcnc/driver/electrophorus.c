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

#define f_period_s ((double)(l_period_ns / 1e9))

typedef struct {
  struct {
    struct {
      hal_float_t *position_cmd;  // in: position command (position units)
      hal_s32_t *counts;          // out: position feedback (raw counts)
      hal_float_t *position_fb;   // out: position feedback (position units)
      hal_bit_t *enable;          // is the stepper enabled?
    } pin;

    struct {
      hal_float_t position_scale;  // steps per position unit
      hal_float_t maxvel;          // max velocity, (pos units/sec)
      hal_float_t maxaccel;        // max accel (pos units/sec^2)
    } param;
  } hal;

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
  bool config_sent;                       // track if configuration has been sent to PRU
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

static void send_config();
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

  // Initialize state variables
  state->config_sent = false;
  state->spi_reset_old = false;
  for (int i = 0; i < STEPGENS; i++) {
    state->stepgens[i].subcounts = 0;
    state->stepgens[i].prev_accumulator = 0;
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

    if (pin_err(hal_pin_float_newf(HAL_IN, &pin->position_cmd, comp_id, "%s.stepgen.%s.position-cmd", prefix, name)))
      return -1;
    *pin->position_cmd = 0.0;

    if (pin_err(hal_pin_float_newf(HAL_OUT, &pin->position_fb, comp_id, "%s.stepgen.%s.position-fb", prefix, name)))
      return -1;
    *pin->position_fb = 0.0;

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
  rtapi_snprintf(name, sizeof(name), "%s.write", prefix);
  /* no FP operations */
  int retval = hal_export_funct(name, spi_write, 0, 0, 0, comp_id);
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

static void send_config() {
  // Send configuration to PRU
  tx_data.header = PRU_CONF;

  for (int i = 0; i < STEPGENS; i++) {
    stepper_state_t *s = &state->stepgens[i];

    // Convert to 24.8 fixed-point format
    tx_data.stepper_position_scale[i] = (uint32_t)(fabs(s->hal.param.position_scale) * 256.0);
    tx_data.stepper_max_accel[i] = (uint32_t)(s->hal.param.maxaccel * 256.0);
    tx_data.stepper_init_position[i] = (uint32_t)(*s->hal.pin.position_cmd * 256.0);
  }

  if (*state->spi_status) {
    spi_transfer();
    state->config_sent = true;
    rtapi_print_msg(RTAPI_MSG_INFO, "%s: Configuration sent to PRU\n", modname);
  }
}



void spi_read() {
  // Data header
  tx_data.header = PRU_READ;

  if (*(state->spi_enable)) {
    // Try SPI communication on reset rising edge OR when status is already good
    if ((*state->spi_reset && !state->spi_reset_old) || *state->spi_status) {
      // Transfer to and from the PRU
      spi_transfer();

      switch (rx_data.header)  // only process valid SPI payloads. This rejects bad payloads
      {
        case PRU_DATA:
          // we have received a GOOD payload from the PRU
          *state->spi_status = 1;

          for (int i = 0; i < STEPGENS; i++) {
            stepper_state_t *s = &state->stepgens[i];

            // PRU sends position feedback directly in machine units
            const fixp_t position_fb_raw = rx_data.stepper_position_fb[i];

            // Convert from fixed-point to machine units
            *s->hal.pin.position_fb = (double)position_fb_raw / FIXED_ONE;

            // Calculate counts for compatibility (position in steps)
            *s->hal.pin.counts = (int32_t)(*s->hal.pin.position_fb * s->hal.param.position_scale);
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
    // SPI disabled, clear status
    *state->spi_status = 0;
  }

  // Reset config_sent flag on reset rising edge to force reconfiguration
  if (*state->spi_reset && !state->spi_reset_old) {
    state->config_sent = false;
  }

  state->spi_reset_old = *state->spi_reset;
}

void spi_write() {
  // Send configuration first if not yet sent
  if (!state->config_sent && *state->spi_status) {
    send_config();
    return;
  }

  // Normal operation - send position commands
  tx_data.header = PRU_WRITE;

  // Send position commands to PRU (PRU handles position scaling)
  for (int i = 0; i < STEPGENS; i++) {
    const stepper_state_t *s = &state->stepgens[i];
    tx_data.stepper_pos_command[i] = (fixp_t)(*s->hal.pin.position_cmd * FIXED_ONE);

    if (*s->hal.pin.enable == 1) {
      tx_data.stepper_enable_mask |= 1 << i;
    } else {
      tx_data.stepper_enable_mask &= ~(1 << i);
    }
  }

  for (int i = 0; i < OUTPUT_VARS; i++) {
    tx_data.output_vars[i] = (int32_t)*state->output_vars[i];
  }

  for (int i = 0; i < OUTPUT_PINS; i++) {
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
