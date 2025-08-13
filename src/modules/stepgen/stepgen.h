#ifndef STEPGEN_H
#define STEPGEN_H

#include "module.h"
#include "pin.h"
#include "spi_comms.h"

class Stepgen final : public Module {
  int joint_enable_mask;

  volatile int32_t *commanded_frequency;  // pointer to the data source where to get the frequency command
  volatile int32_t *feedback;             // pointer where to put the feedback
  volatile uint8_t *joint_enable;

  bool current_dir = true;   // direction on last iteration, used for dir setup
  bool is_stepping = false;  // true if the step pin is held high
  int32_t last_commanded_frequency = 0;
  int32_t step_count = 0;
  int32_t increment = 0;
  int32_t accumulator = 0;  // Direct Digital Synthesis (DDS) accumulator
  uint32_t frequency_scale;

 public:
  Stepgen(int joint_number, Pin *step_pin, Pin *dir_pin, uint32_t thread_frequency, volatile rxData_t *rx_data,
          volatile txData_t *tx_data);

  virtual ~Stepgen() = default;

  Pin *step_pin, *dir_pin;

  void run() override;
};

#endif
