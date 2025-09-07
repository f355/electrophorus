#include "pulse_counter.h"

PulseCounter::PulseCounter(const int var_number, const Pin* pin, SerialComms* comms)
    : comms(comms), var_number(var_number) {
  NVIC_SetPriority(EINT3_IRQn, PIN_IRQ_PRIORITY);
  (new InterruptIn(pin->to_pin_name()))->rise(callback(this, &PulseCounter::interrupt_handler));
}

void PulseCounter::on_rx() { this->comms->get_pru_state()->input_vars[var_number] = this->counter; }

void PulseCounter::interrupt_handler() { this->counter++; }

bool PulseCounter::listens_to_rx() { return true; }
