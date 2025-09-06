#ifndef ADC_H
#define ADC_H

#include "module.h"
#include "pin.h"
#include "spi_comms.h"

class ADC final : public Module {
  volatile int32_t* variable;  // pointer where to put the feedback
  AnalogIn* adc;

  uint32_t run_every;
  uint32_t counter;

 public:
  ADC(int var_number, Pin* pin, volatile txData_t* tx_data);

  void run_servo() override;
};

#endif
