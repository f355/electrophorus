#ifndef PULSECOUNTER_H
#define PULSECOUNTER_H

#include "comms.h"
#include "module.h"
#include "pin.h"

class PulseCounter final : public Module {
  volatile int32_t* variable;
  volatile int32_t counter = 0;
  void interrupt_handler();

 public:
  PulseCounter(int var_number, const Pin* pin, volatile txData_t* tx_data);

  void run() override;
};

#endif
