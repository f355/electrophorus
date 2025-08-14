#ifndef DIGITALOUTPUTS_H
#define DIGITALOUTPUTS_H

#include "module.h"
#include "spi_comms.h"

class DigitalOuts final : public Module {
  volatile uint16_t* outputs;

  uint8_t num_pins;
  LPC_GPIO_TypeDef** ports;
  uint32_t* pin_masks;
  uint16_t invert_mask;

 public:
  DigitalOuts(uint8_t num_pins, const outputPin_t pins[], volatile rxData_t* rx_data);

  void run() override;
};
#endif
