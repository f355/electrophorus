#include "pulse_counter.h"

#include "mbed.h"
#include "pin.h"
#include "spi_comms.h"

PulseCounter::PulseCounter(const uint8_t var_number, const Pin* pin) : var_number_(var_number) {
  (new InterruptIn(pin->ToPinName()))->rise(callback(this, &PulseCounter::InterruptHandler));
}

void PulseCounter::OnRx() { SpiComms::Instance()->get_pru_state()->input_vars[var_number_] = counter_; }

void PulseCounter::InterruptHandler() { counter_++; }
