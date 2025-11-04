#include "electrophorus.h"

#include "rtapi_app.h"

MODULE_AUTHOR("Konstantin Tcepliaev <f355@f355.org>");
MODULE_DESCRIPTION("Driver for the Carvera family of desktop milling machines");
MODULE_LICENSE("GPL v3");

static int spi_freq = 5000000;
RTAPI_MP_INT(spi_freq, "SPI clock frequency in Hz (default 5,000,000)");

int Electrophorus::Init() {
  comp_id_ = hal_init(modname_);
  if (comp_id_ < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s ERROR: hal_init() failed\n", modname_);
    return -1;
  }

  // Allocate HAL-shared memory to hold addresses of pin pointers
  pin_ = static_cast<Pins *>(hal_malloc(sizeof(Pins)));
  if (!pin_) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: hal_malloc() failed for pin pointer container\n", modname_);
    hal_exit(comp_id_);
    return -1;
  }

  spi_reset_old_ = false;
  last_packet_seen_ = 0;
  spi_ = SpiDriver::Detect(spi_freq);
  if (!spi_) {
    rtapi_print_msg(RTAPI_MSG_ERR, "Error: no SPI driver available. Only Raspberry Pi 4 and 5 are supported.\n");
    return -1;
  }
  if (PinErr(hal_pin_bit_newf(HAL_IN, &pin_->spi_enable, comp_id_, "%s.spi-enable", prefix_))) return -1;
  if (PinErr(hal_pin_bit_newf(HAL_IN, &pin_->spi_reset, comp_id_, "%s.spi-reset", prefix_))) return -1;
  if (PinErr(hal_pin_bit_newf(HAL_OUT, &pin_->spi_status, comp_id_, "%s.spi-status", prefix_))) return -1;

  for (int i = 0; i < kNumStepgens; ++i) {
    if (stepgens_[i].Init(comp_id_, modname_, prefix_, i) < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: stepgen[%d] init failed\n", modname_, i);
      hal_exit(comp_id_);
      return -1;
    }
  }
  for (int i = 0; i < kNumOutputVars; i++) {
    if (PinErr(hal_pin_float_newf(HAL_IN, &pin_->output_vars[i], comp_id_, "%s.output-var.%s", prefix_,
                                  kOutputVarNames[i])))
      return -1;
    *pin_->output_vars[i] = 0;
  }
  for (int i = 0; i < kNumInputVars; i++) {
    if (PinErr(
            hal_pin_float_newf(HAL_OUT, &pin_->input_vars[i], comp_id_, "%s.input-var.%s", prefix_, kInputVarNames[i])))
      return -1;
    *pin_->input_vars[i] = 0;
  }
  for (int i = 0; i < kNumOutputPins; i++) {
    if (PinErr(hal_pin_bit_newf(HAL_IN, &pin_->outputs[i], comp_id_, "%s.output.%s", prefix_, kOutputPins[i].name)))
      return -1;
    *pin_->outputs[i] = false;
  }
  for (int i = 0; i < kNumInputPins; i++) {
    if (PinErr(hal_pin_bit_newf(HAL_OUT, &pin_->inputs[i], comp_id_, "%s.input.%s", prefix_, kInputPins[i].name)))
      return -1;
    *pin_->inputs[i] = false;
    if (PinErr(hal_pin_bit_newf(HAL_OUT, &pin_->inputs[i + kNumInputPins], comp_id_, "%s.input.%s.not", prefix_,
                                kInputPins[i].name)))
      return -1;
    *pin_->inputs[i + kNumInputPins] = true;
  }
  char name[HAL_NAME_LEN + 1];
  rtapi_snprintf(name, sizeof(name), "%s.update-freq", prefix_);
  if (hal_export_funct(
          name, [](void *a, const long p) { static_cast<Electrophorus *>(a)->UpdateFreq(p); }, this, 1, 0, comp_id_) <
      0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: update function export failed\n", modname_);
    hal_exit(comp_id_);
    return -1;
  }
  rtapi_snprintf(name, sizeof(name), "%s.write", prefix_);
  if (hal_export_funct(
          name, [](void *a, const long p) { static_cast<Electrophorus *>(a)->Write(p); }, this, 0, 0, comp_id_) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: write function export failed\n", modname_);
    hal_exit(comp_id_);
    return -1;
  }
  rtapi_snprintf(name, sizeof(name), "%s.read", prefix_);
  if (hal_export_funct(
          name, [](void *a, const long p) { static_cast<Electrophorus *>(a)->Read(p); }, this, 1, 0, comp_id_) < 0) {
    rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: read function export failed\n", modname_);
    hal_exit(comp_id_);
    return -1;
  }
  rtapi_print_msg(RTAPI_MSG_INFO, "%s: installed driver\n", modname_);
  hal_ready(comp_id_);
  return 0;
}

void Electrophorus::UpdateFreq(const long period_ns) {
  const double period_s = static_cast<double>(period_ns) * 1e-9;
  linuxcnc_state_->stepgen_dir_mask = 0;
  for (auto &stepgen : stepgens_) stepgen.Update(period_s, linuxcnc_state_);
}

void Electrophorus::Read(const long period_ns) {
  (void)period_ns;
  const bool reset_edge = (*pin_->spi_reset && !spi_reset_old_);
  spi_reset_old_ = *pin_->spi_reset;
  if (!*pin_->spi_enable || (!reset_edge && !*pin_->spi_status)) {
    *pin_->spi_status = false;
    return;
  }
  linuxcnc_state_->command = SpiCommand::Read;
  SpiTransfer();
  const uint32_t cnt = pru_state_->packet_counter;
  bool reset_feedback = false;
  bool publish_inputs = false;
  if (reset_edge) {
    if (last_packet_seen_ != 0 && cnt == kInitCounter) {
      rtapi_print_msg(RTAPI_MSG_ERR, "PRU reset while in e-stop: re-home if axes moved.\n");
    }
    *pin_->spi_status = true;
    reset_feedback = true;
    publish_inputs = true;
  } else if (last_packet_seen_ + 1u == cnt) {
    *pin_->spi_status = true;
    for (auto &stepgen : stepgens_) stepgen.ApplyFeedback(pru_state_);
    publish_inputs = true;
  } else {
    *pin_->spi_status = false;
    reset_feedback = true;
  }
  last_packet_seen_ = cnt;
  if (reset_feedback) {
    for (auto &stepgen : stepgens_) stepgen.ResetPrevFeedback(pru_state_);
  }
  if (publish_inputs) {
    for (int i = 0; i < kNumInputVars; i++) *pin_->input_vars[i] = pru_state_->input_vars[i];
    for (int i = 0; i < kNumInputPins; i++) {
      *pin_->inputs[i] = (pru_state_->inputs & (1 << i)) != 0;
      *pin_->inputs[i + kNumInputPins] = !*pin_->inputs[i];
    }
  }
}

void Electrophorus::Write(const long period_ns) {
  (void)period_ns;
  linuxcnc_state_->command = SpiCommand::Write;
  for (int i = 0; i < kNumOutputVars; i++)
    linuxcnc_state_->output_vars[i] = static_cast<int32_t>(*pin_->output_vars[i]);
  for (int i = 0; i < kNumOutputPins; i++) {
    if (*pin_->outputs[i] == 1)
      linuxcnc_state_->outputs |= 1 << i;
    else
      linuxcnc_state_->outputs &= ~(1 << i);
  }
  if (*pin_->spi_status) SpiTransfer();
}

static Electrophorus *g_comp = nullptr;

extern "C" int rtapi_app_main(void) {
  static Electrophorus comp;
  g_comp = &comp;
  return comp.Init();
}
extern "C" void rtapi_app_exit(void) { /* no-op: process lifetime is one-shot until LinuxCNC exit */ }
