#include "electrophorus.h"

#include "rtapi_app.h"

MODULE_AUTHOR("Konstantin Tcepliaev <f355@f355.org>");
MODULE_DESCRIPTION("Driver for the Carvera family of desktop milling machines");
MODULE_LICENSE("GPL v3");

static int spi_freq = 5000000;
RTAPI_MP_INT(spi_freq, "SPI clock frequency in Hz (default 5,000,000)");

int Electrophorus::init() {
  comp_id = hal_init(modname);
  if (comp_id < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s ERROR: hal_init() failed\n", modname);
    return -1;
  }

  // Allocate HAL-shared memory to hold addresses of pin pointers
  pin = static_cast<Pins *>(hal_malloc(sizeof(Pins)));
  if (!pin) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed for pin pointer container\n", modname);
    hal_exit(comp_id);
    return -1;
  }

  spi_reset_old = false;
  last_packet_seen = 0;
  spi = SpiDriver::detect(spi_freq);
  if (!spi) {
    rtapi_print_msg(RTAPI_MSG_ERR, "Error: no SPI driver available. Only Raspberry Pi 4 and 5 are supported.\n");
    return -1;
  }
  if (pin_err(hal_pin_bit_newf(HAL_IN, &pin->spi_enable, comp_id, "%s.spi-enable", prefix))) return -1;
  if (pin_err(hal_pin_bit_newf(HAL_IN, &pin->spi_reset, comp_id, "%s.spi-reset", prefix))) return -1;
  if (pin_err(hal_pin_bit_newf(HAL_OUT, &pin->spi_status, comp_id, "%s.spi-status", prefix))) return -1;

  for (int i = 0; i < STEPGENS; ++i) {
    if (stepgens[i].init(comp_id, modname, prefix, i) < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: stepgen[%d] init failed\n", modname, i);
      hal_exit(comp_id);
      return -1;
    }
  }
  for (int i = 0; i < OUTPUT_VARS; i++) {
    const char *names[OUTPUT_VARS] = OUTPUT_VAR_NAMES;
    if (pin_err(hal_pin_float_newf(HAL_IN, &pin->output_vars[i], comp_id, "%s.output-var.%s", prefix, names[i])))
      return -1;
    *pin->output_vars[i] = 0;
  }
  for (int i = 0; i < INPUT_VARS; i++) {
    const char *names[INPUT_VARS] = INPUT_VAR_NAMES;
    if (pin_err(hal_pin_float_newf(HAL_OUT, &pin->input_vars[i], comp_id, "%s.input-var.%s", prefix, names[i])))
      return -1;
    *pin->input_vars[i] = 0;
  }
  for (int i = 0; i < OUTPUT_PINS; i++) {
    const outputPin_t pins[OUTPUT_PINS] = OUTPUT_PIN_DESC;
    if (pin_err(hal_pin_bit_newf(HAL_IN, &pin->outputs[i], comp_id, "%s.output.%s", prefix, pins[i].name))) return -1;
    *pin->outputs[i] = false;
  }
  for (int i = 0; i < INPUT_PINS; i++) {
    const inputPin_t pins[INPUT_PINS] = INPUT_PIN_DESC;
    if (pin_err(hal_pin_bit_newf(HAL_OUT, &pin->inputs[i], comp_id, "%s.input.%s", prefix, pins[i].name))) return -1;
    *pin->inputs[i] = false;
    if (pin_err(
            hal_pin_bit_newf(HAL_OUT, &pin->inputs[i + INPUT_PINS], comp_id, "%s.input.%s.not", prefix, pins[i].name)))
      return -1;
    *pin->inputs[i + INPUT_PINS] = false;
  }
  char name[HAL_NAME_LEN + 1];
  rtapi_snprintf(name, sizeof(name), "%s.update-freq", prefix);
  if (hal_export_funct(
          name, [](void *a, const long p) { static_cast<Electrophorus *>(a)->updateFreq(p); }, this, 1, 0, comp_id) <
      0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: update function export failed\n", modname);
    hal_exit(comp_id);
    return -1;
  }
  rtapi_snprintf(name, sizeof(name), "%s.write", prefix);
  if (hal_export_funct(
          name, [](void *a, const long p) { static_cast<Electrophorus *>(a)->write(p); }, this, 0, 0, comp_id) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: write function export failed\n", modname);
    hal_exit(comp_id);
    return -1;
  }
  rtapi_snprintf(name, sizeof(name), "%s.read", prefix);
  if (hal_export_funct(
          name, [](void *a, const long p) { static_cast<Electrophorus *>(a)->read(p); }, this, 1, 0, comp_id) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: read function export failed\n", modname);
    hal_exit(comp_id);
    return -1;
  }
  rtapi_print_msg(RTAPI_MSG_INFO, "%s: installed driver\n", modname);
  hal_ready(comp_id);
  return 0;
}

void Electrophorus::updateFreq(const long period_ns) {
  const double period_s = static_cast<double>(period_ns) * 1e-9;
  linuxcnc_state->stepgen_dir_mask = 0;
  for (auto &stepgen : stepgens) stepgen.update(period_s, linuxcnc_state);
}

void Electrophorus::read(const long period_ns) {
  (void)period_ns;
  const bool reset_edge = (*pin->spi_reset && !spi_reset_old);
  spi_reset_old = *pin->spi_reset;
  if (!*pin->spi_enable || (!reset_edge && !*pin->spi_status)) {
    *pin->spi_status = false;
    return;
  }
  linuxcnc_state->command = SpiCommand::Read;
  spi_transfer();
  const uint32_t cnt = pru_state->packet_counter;
  bool reset_feedback = false;
  bool publish_inputs = false;
  if (reset_edge) {
    if (last_packet_seen != 0 && cnt == PRU_INIT_PKT) {
      rtapi_print_msg(RTAPI_MSG_ERR, "PRU reset while in e-stop: re-home if axes moved.\n");
    }
    *pin->spi_status = true;
    reset_feedback = true;
    publish_inputs = true;
  } else if (last_packet_seen + 1u == cnt) {
    *pin->spi_status = true;
    for (auto &stepgen : stepgens) stepgen.apply_feedback(pru_state);
    publish_inputs = true;
  } else {
    *pin->spi_status = false;
    reset_feedback = true;
  }
  last_packet_seen = cnt;
  if (reset_feedback) {
    for (auto &stepgen : stepgens) stepgen.reset_prev_feedback(pru_state);
  }
  if (publish_inputs) {
    for (int i = 0; i < INPUT_VARS; i++) *pin->input_vars[i] = pru_state->input_vars[i];
    for (int i = 0; i < INPUT_PINS; i++) {
      if ((pru_state->inputs & (1 << i)) != 0) {
        *pin->inputs[i] = true;
        *pin->inputs[i + INPUT_PINS] = false;
      } else {
        *pin->inputs[i] = false;
        *pin->inputs[i + INPUT_PINS] = true;
      }
    }
  }
}

void Electrophorus::write(const long period_ns) {
  (void)period_ns;
  linuxcnc_state->command = SpiCommand::Write;
  for (int i = 0; i < OUTPUT_VARS; i++) linuxcnc_state->output_vars[i] = static_cast<int32_t>(*pin->output_vars[i]);
  for (int i = 0; i < OUTPUT_PINS; i++) {
    if (*pin->outputs[i] == 1)
      linuxcnc_state->outputs |= 1 << i;
    else
      linuxcnc_state->outputs &= ~(1 << i);
  }
  if (*pin->spi_status) spi_transfer();
}

static Electrophorus *g_comp = nullptr;

extern "C" int rtapi_app_main(void) {
  static Electrophorus comp;
  g_comp = &comp;
  return comp.init();
}
extern "C" void rtapi_app_exit(void) { /* no-op: process lifetime is one-shot until LinuxCNC exit */ }
