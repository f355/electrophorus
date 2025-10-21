#include "adc.h"

ADC::ADC(const uint8_t var_number, Pin* pin, SpiComms* comms)
    : comms(comms),
      var_number(var_number),
      adc(new AnalogIn(pin->as_input()->to_pin_name())),
      run_every(100),  // run every 100 calls (0.1 seconds)
      counter(run_every) {
  // Take a reading to get the ADC up and running before moving on
  this->value = this->adc->read_u16();
}

void ADC::on_rx() {
  if (--this->counter == 0) {
    this->counter = run_every;
    this->value = this->adc->read_u16();
  }
  this->comms->get_pru_state()->input_vars[var_number] = this->value;
}
