#include "stepgen.h"

Stepgen::Stepgen(const uint8_t stepgen_number, Pin* step_pin, Pin* dir_pin, const uint32_t ticker_frequency,
                 SpiComms* comms)
    : comms(comms),
      stepgen_number(stepgen_number),
      stepgen_enable_mask(1 << stepgen_number),
      ticker_frequency(ticker_frequency),
      step_pin(step_pin->as_output()),
      dir_pin(dir_pin->as_output()) {
  this->dir_pin->set(false);
}

void Stepgen::make_steps() {
  if (this->is_stepping) {
    // bring down the step pin that was set high on the previous tick
    this->step_pin->set(false);
    this->is_stepping = false;
  }

  if (this->dir_flipped) {
    // the direction of travel has changed, do not step this iteration to give some setup time.
    // TODO(f355): make hold time configurable.
    this->dir_flipped = false;
    return;
  }

  if ((this->comms->get_linuxcnc_state()->stepgen_enable_mask & this->stepgen_enable_mask) == 0) {
    return;  // stepper is disabled, nothing to do
  }

  // Direct Digital Synthesis (DDS)
  // works by incrementing a fixed-point accumulator on every tick with a value calculated such that the accumulator
  // reaches fixed-point 1.0 at the commanded frequency.
  //
  // That value is, unsurprisingly, the fixed-point commanded frequency divided by the integer ticker frequency - to get
  // steps/tick, you divide steps/second by ticks/second.

  const int32_t inc = this->increment;  // Q0.32
  if (inc == 0) return;

  const uint32_t old_sub = this->substeps;
  this->substeps = old_sub + static_cast<uint32_t>(inc);
  if (inc >= 0) {
    if (this->substeps < old_sub) {
      this->steps += 1;
      this->step_pin->set(true);
      this->is_stepping = true;
    }
  } else {
    if (this->substeps > old_sub) {
      this->steps -= 1;
      this->step_pin->set(true);
      this->is_stepping = true;
    }
  }

  const int64_t pos = (this->steps << 32) + static_cast<uint32_t>(this->substeps);
  this->comms->get_pru_state()->stepgen_feedback[this->stepgen_number] = pos;
}

void Stepgen::on_rx() {
  const float commanded_frequency = this->comms->get_linuxcnc_state()->stepgen_freq_command[this->stepgen_number];
  if ((this->comms->get_linuxcnc_state()->stepgen_enable_mask & this->stepgen_enable_mask) == 0 ||
      this->last_commanded_frequency == commanded_frequency) {
    return;  // stepper is disabled or the command hasn't changed, nothing to do
  }
  this->last_commanded_frequency = commanded_frequency;

  // the commanded frequency has changed, recalculate the increment
  const auto new_inc =
      static_cast<int32_t>(commanded_frequency * (static_cast<float>(FIXED_ONE) / this->ticker_frequency));
  const bool prev_dir = this->increment > 0;
  this->increment = new_inc;

  // The sign of the increment indicates the desired direction
  if (const bool new_dir = new_inc > 0; prev_dir != new_dir) {
    this->dir_flipped = true;
    this->dir_pin->set(new_dir);
  }
}

bool Stepgen::is_stepgen() { return true; }
