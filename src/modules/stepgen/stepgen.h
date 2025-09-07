#ifndef STEPGEN_H
#define STEPGEN_H

#include "module.h"
#include "pin.h"
#include "serial_comms.h"

class Stepgen final : public Module {
  const SerialComms *comms;
  int stepper_number;
  int stepper_enable_mask;

  uint32_t ticker_frequency;
  volatile bool dir_flipped = false;  // indicates that the direction has changed
  bool current_dir = true;            // direction on last iteration, used for dir setup
  bool is_stepping = false;           // true if the step pin is held high
  float last_commanded_frequency = 0;
  int64_t step_position;
  volatile int64_t increment = 0;

 public:
  Stepgen(int stepper_number, Pin *step_pin, Pin *dir_pin, uint32_t ticker_frequency, const SerialComms *comms);

  ~Stepgen() = default;

  Pin *step_pin, *dir_pin;

  bool listens_to_rx() override;
  bool is_stepgen() override;
  void make_steps() override;
  void on_rx() override;
};

#endif
