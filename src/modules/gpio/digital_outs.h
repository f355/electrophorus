#ifndef DIGITALOUTPUTS_H
#define DIGITALOUTPUTS_H

#include "modules/module.h"
#include "spi_comms.h"

class DigitalOuts final : public Module {
  SpiComms* comms;

  uint8_t num_pins;
  LPC_GPIO_TypeDef** ports;
  uint32_t* pin_masks;
  uint16_t invert_mask;

  uint16_t last_outputs = 1;  // 1 just so it is not zero

 public:
  DigitalOuts(uint8_t num_pins, const outputPin_t pins[], SpiComms* comms);

  void on_rx() override;
};
#endif
