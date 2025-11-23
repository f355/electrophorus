#pragma once

#include "modules/module.h"
#include "pin.h"
#include "spi_comms.h"

class Stepgen final : public Module {
  uint8_t stepgen_number_;

  Pin *step_pin_, *dir_pin_;

  uint32_t substeps_ = 0;                    // Q0.32 fractional substeps
  uint16_t steps_ = 0;                       // whole steps mod 2^16, updated on substeps wrap
  volatile uint32_t substeps_per_tick_ = 0;  // commanded substeps per base tick (magnitude)
  bool current_dir_ = true;                  // direction we're moving in
  volatile bool dir_flipped_ = false;        // indicates that the direction has changed

 public:
  Stepgen(uint8_t stepgen_number, Pin *step_pin, Pin *dir_pin);

  ~Stepgen() = default;

  bool IsStepgen() override;
  void MakeSteps() override;
  void OnRx() override;
};
