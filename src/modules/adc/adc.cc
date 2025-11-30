#include "adc.h"

#include "mbed.h"
#include "pin.h"
#include "spi_comms.h"

ADC::ADC(const uint8_t var_number, Pin* pin)
    : var_number_(var_number),
      adc_(new AnalogIn(pin->AsInput()->ToPinName())),
      run_every_(100),  // run every 100 calls (0.1 seconds)
      counter_(run_every_) {
  // Take a reading to get the ADC up and running before moving on
  value_ = adc_->read_u16();
}

void ADC::OnRx() {
  if (--counter_ == 0) {
    counter_ = run_every_;
    value_ = adc_->read_u16();
  }
  SpiComms::Instance()->get_pru_state()->input_vars[var_number_] = value_;
}
