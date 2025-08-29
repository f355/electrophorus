#ifndef STEPGEN_H
#define STEPGEN_H

#include "module.h"
#include "pin.h"
#include "spi_comms.h"

class Stepgen final : public Module {
  int stepper_enable_mask;

  volatile fixp_t *commanded_frequency;  // frequency command, 32.32 fixed-point
  volatile fixp_t *step_position;        // 32.32 fixed-point number of steps taken,
                                         // serves as both DDS accumulator and position feedback
  volatile uint8_t *stepper_enable;

  int64_t ticker_frequency;
  bool current_dir = true;   // direction on last iteration, used for dir setup
  bool is_stepping = false;  // true if the step pin is held high
  fixp_t last_commanded_frequency = 0;
  int64_t increment = 0;

 public:
  Stepgen(int stepper_number, Pin *step_pin, Pin *dir_pin, uint32_t ticker_frequency, volatile rxData_t *rx_data,
          volatile txData_t *tx_data);

  virtual ~Stepgen() = default;

  Pin *step_pin, *dir_pin;

  void run() override;
};

#endif
