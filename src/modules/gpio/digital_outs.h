#ifndef DIGITALOUTPUTS_H
#define DIGITALOUTPUTS_H

#include "module.h"
#include "serial_comms.h"

class DigitalOuts final : public Module {
  const SerialComms* comms;

  uint8_t num_pins;
  LPC_GPIO_TypeDef** ports;
  uint32_t* pin_masks;
  uint16_t invert_mask;

 public:
  DigitalOuts(uint8_t num_pins, const outputPin_t pins[], const SerialComms* comms);

  bool listens_to_rx() override;
  void on_rx() override;
};
#endif
