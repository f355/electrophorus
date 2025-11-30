#pragma once

#include "LPC17xx.h"
#include "modules/module.h"
#include "spi_protocol/machine_definitions.h"

class DigitalOuts final : public Module {
  uint8_t num_pins_;
  LPC_GPIO_TypeDef** ports_;
  uint32_t* pin_masks_;
  uint16_t invert_mask_;

  uint16_t last_outputs_ = 1;  // 1 just so it is not zero

 public:
  DigitalOuts(uint8_t num_pins, const outputPin_t pins[]);

  void OnRx() override;
};
