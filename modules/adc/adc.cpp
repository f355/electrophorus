#include "adc.h"

ADC::ADC(const int var_number, Pin* pin, volatile txData_t* tx_data)
    : variable(&tx_data->input_vars[var_number]), adc() {
  // Take some readings to get the ADC up and running before moving on
  analogin_init(&this->adc, pin->as_input()->to_pin_name());
  this->run();
  this->run();
}

void ADC::run() { *this->variable = analogin_read_u16(&this->adc); }
