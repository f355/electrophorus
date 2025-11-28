#include "e_stop.h"

#include "spi_comms.h"

EStop::EStop(const Pin* pin) {
  const auto irqPin = new InterruptIn(pin->ToPinName());
  if (pin->inverting_) {
    irqPin->fall(callback(&EStop::Engaged));
    irqPin->rise(callback(&EStop::Disengaged));
  } else {
    irqPin->rise(callback(&EStop::Engaged));
    irqPin->fall(callback(&EStop::Disengaged));
  }
  if (pin->Get()) Engaged();
}

void EStop::Engaged() { SpiComms::Instance()->EStop(true); }

void EStop::Disengaged() { SpiComms::Instance()->EStop(false); }
