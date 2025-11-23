#include "stepgen.h"

Stepgen::Stepgen(const uint8_t stepgen_number, Pin* step_pin, Pin* dir_pin)
    : stepgen_number_(stepgen_number), step_pin_(step_pin->AsOutput()), dir_pin_(dir_pin->AsOutput()) {
  dir_pin_->Set(current_dir_);
}

void Stepgen::MakeSteps() {
  // bring down the step pin that was set high on the previous tick
  step_pin_->Set(false);

  // Direct Digital Synthesis (DDS): advance substeps and emit a pulse on wrap
  if (substeps_per_tick_ == 0) return;  // zero steps/tick disables stepping

  if (dir_flipped_) {
    // the direction of travel has changed, do not step this iteration to give some setup time.
    // TODO(f355): make hold time configurable.
    dir_flipped_ = false;
    return;
  }

  const uint32_t old = substeps_;
  if (current_dir_) {
    substeps_ += substeps_per_tick_;
    if (substeps_ < old) {
      steps_++;
      step_pin_->Set(true);
    }
  } else {
    substeps_ -= substeps_per_tick_;
    if (substeps_ > old) {
      steps_--;
      step_pin_->Set(true);
    }
  }

  // Pack Q16.16 feedback: high 16 bits are integer steps (mod 2^16), low 16 bits are substeps >> 16
  SpiComms::Instance()->get_pru_state()->stepgen_feedback[stepgen_number_] =
      static_cast<int32_t>(static_cast<uint32_t>(steps_) << 16 | substeps_ >> 16);
}

void Stepgen::OnRx() {
  const uint32_t inc = SpiComms::Instance()->get_linuxcnc_state()->steps_per_tick_cmd[stepgen_number_];
  substeps_per_tick_ = inc;

  const bool is_forward = (SpiComms::Instance()->get_linuxcnc_state()->stepgen_dir_mask >> stepgen_number_ & 0x1) != 0;
  if (current_dir_ != is_forward) {
    current_dir_ = is_forward;
    dir_flipped_ = true;
    dir_pin_->Set(is_forward);
  }
}

bool Stepgen::IsStepgen() { return true; }
