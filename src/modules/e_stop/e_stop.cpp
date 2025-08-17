#include "e_stop.h"

EStop::EStop(const Pin* pin, SpiComms* comms) : comms(comms) {
  NVIC_SetPriority(EINT3_IRQn, 16);
  const auto irqPin = new InterruptIn(pin->to_pin_name());
  irqPin->rise(callback(this, &EStop::rise_handler));
  irqPin->fall(callback(this, &EStop::fall_handler));
}

void EStop::rise_handler() const {
  this->comms->e_stop_active = true;
  // kill the steppers
  this->comms->rx_data->stepper_enable = 0;
  // kill the spindle (assumes the spindle speed is the first output_var)
  this->comms->rx_data->output_vars[0] = 0;
}

void EStop::fall_handler() const { this->comms->e_stop_active = false; }
