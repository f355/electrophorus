#ifndef PULSECOUNTER_H
#define PULSECOUNTER_H

#include "module.h"
#include "pin.h"
#include "spi_comms.h"

class PulseCounter final : public Module {
  SpiComms* comms;
  uint8_t var_number;

  volatile int32_t counter = 0;
  void interrupt_handler();

 public:
  PulseCounter(uint8_t var_number, const Pin* pin, SpiComms* comms);

  bool is_servo() override;
  void run_servo() override;
};

#endif
