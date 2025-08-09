#include "pulseCounter.h"

PulseCounter::PulseCounter(const int var_number, const Pin* pin, volatile txData_t* tx_data)
    : variable(&tx_data->input_vars[var_number]) {
  (new InterruptIn(pin->to_pin_name()))->rise(callback(this, &PulseCounter::interrupt_handler));
}

void PulseCounter::run() { *this->variable = this->counter; }

void PulseCounter::interrupt_handler() { this->counter++; }
