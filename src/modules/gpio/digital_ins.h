#ifndef DIGITALINPUTS_H
#define DIGITALINPUTS_H

#include "module.h"
#include "serial_comms.h"

class DigitalIns final : public Module {
  const SerialComms* comms;

  uint8_t num_pins;
  LPC_GPIO_TypeDef** ports;
  uint8_t* pins;
  uint16_t invert_mask;

 public:
  DigitalIns(uint8_t num_pins, const inputPin_t pins[], const SerialComms* comms);

  bool listens_to_rx() override;
  void on_rx() override;
};

#endif
