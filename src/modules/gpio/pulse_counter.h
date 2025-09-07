#ifndef PULSECOUNTER_H
#define PULSECOUNTER_H

#include "module.h"
#include "pin.h"
#include "serial_comms.h"

class PulseCounter final : public Module {
  SerialComms* comms;
  int var_number;

  volatile int32_t counter = 0;

  void interrupt_handler();

 public:
  PulseCounter(int var_number, const Pin* pin, SerialComms* comms);

  bool listens_to_rx() override;
  void on_rx() override;
};

#endif
