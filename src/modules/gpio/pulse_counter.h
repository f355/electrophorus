#ifndef PULSECOUNTER_H
#define PULSECOUNTER_H

#include "module.h"
#include "pin.h"
#include "spi_comms.h"

class PulseCounter final : public Module {
  volatile int32_t* variable;
  volatile int32_t counter = 0;
  void interrupt_handler();

 public:
  PulseCounter(int var_number, const Pin* pin, volatile txData_t* tx_data);

  bool is_servo() override;
  void run_servo() override;
};

#endif
