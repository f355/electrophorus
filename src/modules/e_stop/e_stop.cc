#include "e_stop.h"

#include "mbed.h"
#include "pin.h"
#include "spi_comms.h"

EStop::EStop(const Pin* pin) : pin_(pin), initialized_(false) {
  const auto irqPin = new InterruptIn(pin->ToPinName());
  if (pin->inverting_) {
    // normally-closed button
    irqPin->fall(callback(&EStop::Engaged));
    irqPin->rise(callback(&EStop::Disengaged));
  } else {
    // normally-open button
    irqPin->rise(callback(&EStop::Engaged));
    irqPin->fall(callback(&EStop::Disengaged));
  }
}

void EStop::OnRx() {
  if (!initialized_) {
    initialized_ = true;
    if (pin_->Get()) Engaged();
  }
}

void EStop::Engaged() { SpiComms::Instance()->EStop(true); }

void EStop::Disengaged() { SpiComms::Instance()->EStop(false); }
