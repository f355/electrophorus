#ifndef STEPGEN_H
#define STEPGEN_H

#include "module.h"
#include "pin.h"
#include "spi_comms.h"

class Stepgen final : public Module {
  SpiComms *comms;
  uint8_t stepgen_number;
  uint8_t stepgen_enable_mask;

  uint32_t ticker_frequency;
  volatile bool dir_flipped = false;  // indicates that the direction has changed
  bool is_stepping = false;           // true if the step pin is held high
  float last_commanded_frequency = 0;
  int64_t steps = 0;      // integer step count (Q32.32 high)
  uint32_t substeps = 0;  // fractional substeps (Q0.32 low)
  volatile int32_t increment = 0;  // Q0.32 per-tick increment

 public:
  Stepgen(uint8_t stepgen_number, Pin *step_pin, Pin *dir_pin, uint32_t ticker_frequency, SpiComms *comms);

  ~Stepgen() = default;

  Pin *step_pin, *dir_pin;

  bool is_stepgen() override;
  void make_steps() override;
  void on_rx() override;
};

#endif
