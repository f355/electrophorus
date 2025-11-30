#pragma once

#include "LPC17xx.h"
#include "modules/module.h"
#include "spi_protocol/machine_definitions.h"

class DigitalIns final : public Module {
  uint8_t num_pins_;
  LPC_GPIO_TypeDef** ports_;
  uint8_t* pins_;
  uint16_t invert_mask_;

 public:
  DigitalIns(uint8_t num_pins, const inputPin_t pins[]);

  void OnRx() override;
};
