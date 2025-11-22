#include "stepgen.h"

Stepgen::Stepgen(const uint8_t stepgen_number, Pin* step_pin, Pin* dir_pin, SpiComms* comms)
    : comms(comms), stepgen_number(stepgen_number), step_pin(step_pin->as_output()), dir_pin(dir_pin->as_output()) {
  this->dir_pin->set(this->current_dir);
}

void Stepgen::make_steps() {
  // bring down the step pin that was set high on the previous tick
  this->step_pin->set(false);

  // Direct Digital Synthesis (DDS): advance substeps and emit a pulse on wrap
  if (this->substeps_per_tick == 0) return;  // zero steps/tick disables stepping

  if (this->dir_flipped) {
    // the direction of travel has changed, do not step this iteration to give some setup time.
    // TODO(f355): make hold time configurable.
    this->dir_flipped = false;
    return;
  }

  const uint32_t old = this->substeps;
  if (this->current_dir) {
    this->substeps += this->substeps_per_tick;
    if (this->substeps < old) {
      this->steps++;
      this->step_pin->set(true);
    }
  } else {
    this->substeps -= this->substeps_per_tick;
    if (this->substeps > old) {
      this->steps--;
      this->step_pin->set(true);
    }
  }

  // Pack Q16.16 feedback: high 16 bits are integer steps (mod 2^16), low 16 bits are substeps >> 16
  this->comms->get_pru_state()->stepgen_feedback[this->stepgen_number] =
      static_cast<int32_t>(static_cast<uint32_t>(this->steps) << 16 | this->substeps >> 16);
}

void Stepgen::on_rx() {
  const uint32_t inc = this->comms->get_linuxcnc_state()->steps_per_tick_cmd[this->stepgen_number];
  this->substeps_per_tick = inc;

  const bool is_forward = (this->comms->get_linuxcnc_state()->stepgen_dir_mask >> this->stepgen_number & 0x1) != 0;
  if (this->current_dir != is_forward) {
    this->current_dir = is_forward;
    this->dir_flipped = true;
    this->dir_pin->set(is_forward);
  }
}

bool Stepgen::is_stepgen() { return true; }
