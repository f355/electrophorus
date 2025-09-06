#ifndef STEPPER_H
#define STEPPER_H

#include "module.h"
#include "pin.h"
#include "spi_comms.h"

class Stepgen final : public Module {
  // parameters
  uint8_t stepper_number;
  int stepper_enable_mask;
  float max_accel;       // maximum velocity in steps/s/s
  float position_scale;  // number of steps per machine unit

  // state
  volatile rxData_t *rx_data;
  volatile txData_t *tx_data;
  float old_position_command;         // previous position command
  float current_velocity;             // the velocity in steps/s at which the stepper is currently moving
  volatile int64_t step_position;     // DDS accumulator
  volatile int64_t increment;         // DDS increment
  volatile bool configured = false;   // true if the parameters were set
  volatile bool current_dir = true;   // current direction in which the axis is moving
  volatile bool dir_flipped = false;  // indicates if the travel direction has changed
  volatile bool is_stepping = false;  // true if the step pin is held high

  void calc_velocity();

 public:
  Stepgen(int stepper_number, Pin *step_pin, Pin *dir_pin, volatile rxData_t *rx_data, volatile txData_t *tx_data);

  virtual ~Stepgen() = default;

  Pin *step_pin, *dir_pin;

  bool is_base() override { return true; }
  void run_servo() override;
  void run_base() override;
};

#endif  // STEPPER_H
