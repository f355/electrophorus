#ifndef ADC_H
#define ADC_H

#include "module.h"
#include "pin.h"
#include "spi_comms.h"

class ADC final : public Module {
  SpiComms* comms;
  uint8_t var_number;
  AnalogIn* adc;

  uint32_t run_every;
  uint32_t counter;

 public:
  ADC(uint8_t var_number, Pin* pin, SpiComms* comms);

  bool is_servo() override;
  void run_servo() override;
};

#endif
