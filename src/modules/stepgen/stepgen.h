#ifndef STEPGEN_H
#define STEPGEN_H

#include "module.h"
#include "pin.h"
#include "spi_comms.h"

class Stepgen final : public Module {
  int stepper_enable_mask;

  volatile float *commanded_frequency;
  volatile int64_t *step_position;
  volatile uint8_t *stepper_enable;

  uint32_t ticker_frequency;
  bool current_dir = true;   // direction on last iteration, used for dir setup
  bool is_stepping = false;  // true if the step pin is held high
  float last_commanded_frequency = 0;
  int64_t accumulator = 0;
  int64_t increment = 0;

 public:
  Stepgen(int stepper_number, Pin *step_pin, Pin *dir_pin, uint32_t ticker_frequency, volatile rxData_t *rx_data,
          volatile txData_t *tx_data);

  ~Stepgen() = default;

  Pin *step_pin, *dir_pin;

  void run() override;
};

#endif
