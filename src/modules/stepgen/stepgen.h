#ifndef STEPGEN_H
#define STEPGEN_H

#include <cstdint>

#include "machine_definitions.h"
#include "module.h"
#include "pin.h"
#include "spi_comms.h"

class Stepgen final : public Module {
  int stepper_enable_mask;
  int stepper_number;

  volatile int64_t *step_position;
  volatile uint8_t *stepper_enable;
  volatile rxData_t *rx_data;

  uint32_t ticker_frequency;
  volatile bool dir_flipped = false;  // indicates that the direction has changed
  bool current_dir = true;            // direction on last iteration, used for dir setup
  bool is_stepping = false;           // true if the step pin is held high
  float last_commanded_frequency = 0;
  int64_t accumulator = 0;
  volatile int64_t increment = 0;

  // Position control state
  float old_position_cmd = 0;
  float velocity_fb = 0;  // Velocity feedback for control loop
  float maxvel = 0;
  float maxaccel = 0;
  bool config_received = false;  // Safety flag

  static constexpr float SERVO_PERIOD = 1.0f / SERVO_FREQUENCY;

 public:
  Stepgen(int stepper_number, Pin *step_pin, Pin *dir_pin, uint32_t ticker_frequency, volatile rxData_t *rx_data,
          volatile txData_t *tx_data);

  ~Stepgen() = default;

  Pin *step_pin, *dir_pin;

  bool listens_to_rx() override;
  bool is_base() override;
  bool is_servo() override;
  void run_base() override;
  void run_servo() override;
  void on_rx() override;
};

#endif
