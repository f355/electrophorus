#include "adc.h"

ADC::ADC(const int var_number, Pin* pin, volatile txData_t* tx_data)
    : variable(&tx_data->input_vars[var_number]),
      adc(new AnalogIn(pin->as_input()->to_pin_name())),
      run_every(100),  // run every 100 timer ticks (0.1 seconds)
      counter(1) {
  // Take a reading to get the ADC up and running before moving on
  this->ADC::run_servo();
}

void ADC::run_servo() {
  if (--this->counter == 0) {
    this->counter = run_every;
    *this->variable = this->adc->read_u16();
  }
}

bool ADC::is_servo() { return true; }
