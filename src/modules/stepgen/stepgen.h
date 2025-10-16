#ifndef STEPGEN_H
#define STEPGEN_H

#include "modules/module.h"
#include "pin.h"
#include "spi_comms.h"

class Stepgen final : public Module {
  SpiComms *comms;
  uint8_t stepgen_number;

  Pin *step_pin, *dir_pin;

  uint32_t substeps = 0;                    // Q0.32 fractional substeps
  uint16_t steps = 0;                       // whole steps mod 2^16, updated on substeps wrap
  volatile uint32_t substeps_per_tick = 0;  // commanded substeps per base tick (magnitude)
  bool current_dir = true;                  // direction we're moving in
  volatile bool dir_flipped = false;        // indicates that the direction has changed

 public:
  Stepgen(uint8_t stepgen_number, Pin *step_pin, Pin *dir_pin, SpiComms *comms);

  ~Stepgen() = default;

  bool is_stepgen() override;
  void make_steps() override;
  void on_rx() override;
};

#endif
