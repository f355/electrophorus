#ifndef STEPGEN_H
#define STEPGEN_H

#include "modules/module.h"
#include "pin.h"
#include "spi_comms.h"

class Stepgen final : public Module {
  SpiComms *comms;
  uint8_t stepgen_number;
  uint8_t stepgen_enable_mask;

  Pin *step_pin, *dir_pin;

  int64_t position = 0;               // current position in substeps (fractional steps)
  volatile int64_t increment = 0;     // number of substeps to move on each tick
  bool current_dir = true;            // direction we're moving in
  volatile bool dir_flipped = false;  // indicates that the direction has changed
  float last_commanded_frequency = 0;

 public:
  Stepgen(uint8_t stepgen_number, Pin *step_pin, Pin *dir_pin, SpiComms *comms);

  ~Stepgen() = default;

  bool is_stepgen() override;
  void make_steps() override;
  void on_rx() override;
};

#endif
