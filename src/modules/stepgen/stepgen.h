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
  bool current_dir = true;            // direction on last iteration, used for dir setup
  bool is_stepping = false;           // true if the step pin is held high
  float last_commanded_frequency = 0;
  int64_t accumulator = 0;
  volatile int64_t increment = 0;

 public:
  Stepgen(uint8_t stepgen_number, Pin *step_pin, Pin *dir_pin, uint32_t ticker_frequency, SpiComms *comms);

  ~Stepgen() = default;

  Pin *step_pin, *dir_pin;

  bool listens_to_rx() override;
  bool is_base() override;
  void run_base() override;
  void on_rx() override;
};

#endif
