#pragma once

#include <cstdint>

#include "modules/module.h"

// Quadrature encoder using the LPC1768's built-in QEI peripheral.
// Fixed pins: MCI0 = P1.20 (PhA), MCI1 = P1.23 (PhB)
class QuadratureEncoder final : public Module {
  uint8_t position_var_number_;
  uint8_t velocity_var_number_;

 public:
  // velocity_period_us: measurement period for velocity in microseconds (0 to disable velocity)
  QuadratureEncoder(uint8_t position_var_number, uint8_t velocity_var_number, bool count_both_phases,
                    uint32_t velocity_period_us);

  void OnRx() override;
};
