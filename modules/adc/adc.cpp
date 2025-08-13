#include "adc.h"

ADC::ADC(const int var_number, Pin* pin, volatile txData_t* tx_data)
    : variable(&tx_data->input_vars[var_number]),
      adc(new AnalogIn(pin->as_input()->to_pin_name())),
      run_every(20),  // run every 20 timer ticks
      counter(run_every) {
  // Take some readings to get the ADC up and running before moving on
  this->run();
  this->run();
}

void ADC::run() {
  if (--this->counter == 0) {
    this->counter = run_every;
    *this->variable = this->adc->read_u16();
  }
}
