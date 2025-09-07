#include "adc.h"

ADC::ADC(const int var_number, Pin* pin, const SerialComms* comms)
    : comms(comms),
      var_number(var_number),
      adc(new AnalogIn(pin->as_input()->to_pin_name())),
      run_every(99),   // keep this number odd
      counter(1) {
  // Take an initial reading to warm up ADC and publish a value
  this->comms->get_pru_state()->input_vars[var_number] = this->adc->read_u16();
}

void ADC::on_rx() {
  if (--this->counter == 0) {
    this->counter = run_every;
    this->comms->get_pru_state()->input_vars[var_number] = this->adc->read_u16();
  }
}

bool ADC::listens_to_rx() { return true; }
