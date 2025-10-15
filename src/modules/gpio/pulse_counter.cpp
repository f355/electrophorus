#include "pulse_counter.h"

PulseCounter::PulseCounter(const uint8_t var_number, const Pin* pin, SpiComms* comms)
    : comms(comms), var_number(var_number) {
  NVIC_SetPriority(EINT3_IRQn, GPIO_IRQ_PRIORITY);
  (new InterruptIn(pin->to_pin_name()))->rise(callback(this, &PulseCounter::interrupt_handler));
}

void PulseCounter::on_rx() { this->comms->get_pru_state()->input_vars[this->var_number] = this->counter; }

void PulseCounter::interrupt_handler() { this->counter++; }
