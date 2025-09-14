#include "stepgen.h"

Stepgen::Stepgen(const int stepper_number, Pin* step_pin, Pin* dir_pin, const uint32_t ticker_frequency,
                 volatile rxData_t* rx_data, volatile txData_t* tx_data)
    : stepper_enable_mask(1 << stepper_number),
      commanded_frequency(&rx_data->stepgen_freq_command[stepper_number]),
      step_position(&tx_data->stepgen_feedback[stepper_number]),
      stepper_enable(&rx_data->stepgen_enable_mask),
      ticker_frequency(ticker_frequency),
      step_pin(step_pin->as_output()),
      dir_pin(dir_pin->as_output()) {
  this->dir_pin->set(this->current_dir);
}

void Stepgen::run_base() {
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

  if ((*this->stepper_enable & this->stepper_enable_mask) == 0) {
    return;  // stepper is disabled, nothing to do
  }

  // Direct Digital Synthesis (DDS)
  // works by incrementing a fixed-point accumulator on every tick with a value calculated such that the accumulator
  // reaches fixed-point 1.0 at the commanded frequency.
  //
  // That value is, unsurprisingly, the fixed-point commanded frequency divided by the integer ticker frequency - to get
  // steps/tick, you divide steps/second by ticks/second.

  if (this->increment == 0) return;

  int64_t position = *this->step_position;
  const int64_t old_position = position;
  position += increment;

  if ((old_position ^ position) & FIXED_ONE) {
    // the next whole step value is reached, make a step
    this->step_pin->set(true);
    this->is_stepping = true;
  }
  *this->step_position = position;
}

void Stepgen::on_rx() {
  if ((*this->stepper_enable & this->stepper_enable_mask) == 0 ||
      (this->last_commanded_frequency == *this->commanded_frequency)) {
    return;  // stepper is disabled or the command hasn't changed, nothing to do
  }

  // the commanded frequency has changed, recalculate the increment
  this->last_commanded_frequency = *this->commanded_frequency;
  this->increment =
      static_cast<int64_t>(this->last_commanded_frequency * (static_cast<float>(FIXED_ONE) / this->ticker_frequency));

  // The sign of the increment indicates the desired direction
  if (const bool is_forward = increment > 0; this->current_dir != is_forward) {
    this->current_dir = is_forward;
    this->dir_flipped = true;
    this->dir_pin->set(is_forward);
  }
}

bool Stepgen::listens_to_rx() { return true; }

bool Stepgen::is_base() { return true; }
