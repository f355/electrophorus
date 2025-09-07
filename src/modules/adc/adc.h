#ifndef ADC_H
#define ADC_H

#include "module.h"
#include "pin.h"
#include "serial_comms.h"

class ADC final : public Module {
  const SerialComms* comms;
  int var_number;
  AnalogIn* adc;

  uint32_t run_every;
  uint32_t counter;

 public:
  ADC(int var_number, Pin* pin, const SerialComms* comms);

  bool listens_to_rx() override;
  void on_rx() override;
};

#endif
