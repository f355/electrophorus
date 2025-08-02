#include "pulseCounter.h"

PulseCounter::PulseCounter(const int var_number, Pin* pin, volatile txData_t* tx_data)
    : variable(&tx_data->input_vars[var_number]) {
  this->interrupt = pin->interrupt_pin();
  this->interrupt->rise(callback(this, &PulseCounter::interrupt_handler));
}

void PulseCounter::interrupt_handler() { *this->variable++; }
