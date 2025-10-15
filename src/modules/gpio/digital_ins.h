#ifndef DIGITALINPUTS_H
#define DIGITALINPUTS_H

#include "modules/module.h"
#include "spi_comms.h"

class DigitalIns final : public Module {
  SpiComms* comms;

  uint8_t num_pins;
  LPC_GPIO_TypeDef** ports;
  uint8_t* pins;
  uint16_t invert_mask;

 public:
  DigitalIns(uint8_t num_pins, const inputPin_t pins[], SpiComms* comms);

  void on_rx() override;
};

#endif
