#ifndef STEPPER_H
#define STEPPER_H

#include "fixed32_32.h"
#include "module.h"
#include "pin.h"
#include "spi_comms.h"

class Stepper final : public Module {
  // parameters
  uint8_t stepper_number;
  int stepper_enable_mask;
  Fixed32_32 max_accel;       // maximum velocity in steps/s/s
  Fixed32_32 position_scale;  // number of steps per machine unit

  // state
  volatile rxData_t *rx_data;
  volatile txData_t *tx_data;
  Fixed32_32 old_position_command;    // previous position command
  Fixed32_32 current_velocity;        // the velocity in steps/s at which the stepper is currently moving
  volatile Fixed32_32 step_position;  // the current position in steps serves as DDS accumulator
  volatile Fixed32_32 increment;      // DDS increment
  volatile bool configured = false;   // true if the parameters were set
  volatile bool current_dir = true;   // current direction in which the axis is moving
  volatile bool dir_flipped = false;  // indicates if the travel direction has changed
  volatile bool is_stepping = false;  // true if the step pin is held high

  void calc_velocity();

 public:
  Stepper(int stepper_number, Pin *step_pin, Pin *dir_pin, volatile rxData_t *rx_data, volatile txData_t *tx_data);

  virtual ~Stepper() = default;

  Pin *step_pin, *dir_pin;

  bool is_base() override { return true; }
  void run_servo() override;
  void run_base() override;
};

#endif  // STEPPER_H
