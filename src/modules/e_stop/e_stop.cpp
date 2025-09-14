#include "e_stop.h"

EStop::EStop(const Pin* pin, SpiComms* comms) : comms(comms), normally_closed(pin->inverting) {
  NVIC_SetPriority(EINT3_IRQn, 16);
  const auto irqPin = new InterruptIn(pin->to_pin_name());
  irqPin->rise(callback(this, &EStop::rise_handler));
  irqPin->fall(callback(this, &EStop::fall_handler));
  if (pin->get()) {
    this->engaged();
  }
}

void EStop::rise_handler() const {
  if (this->normally_closed) {
    this->disengaged();
  } else {
    this->engaged();
  }
}

void EStop::fall_handler() const {
  if (this->normally_closed) {
    this->engaged();
  } else {
    this->disengaged();
  }
}

// ReSharper disable once CppDFAUnreachableFunctionCall
void EStop::engaged() const {
  this->comms->e_stop_active = true;
  // kill the steppers
  this->comms->rx_data->stepgen_enable_mask = 0;
  // kill the spindle (assumes the spindle speed is the first output_var)
  this->comms->rx_data->output_vars[0] = 0;
}

// ReSharper disable once CppDFAUnreachableFunctionCall
void EStop::disengaged() const { this->comms->e_stop_active = false; }
