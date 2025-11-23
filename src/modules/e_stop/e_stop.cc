#include "e_stop.h"

#include "spi_comms.h"

EStop::EStop(const Pin* pin) : normally_closed_(pin->inverting_) {
  const auto irqPin = new InterruptIn(pin->ToPinName());
  irqPin->rise(callback(this, &EStop::RiseHandler));
  irqPin->fall(callback(this, &EStop::FallHandler));
  if (pin->Get()) {
    Engaged();
  }
}

void EStop::RiseHandler() const {
  if (normally_closed_) {
    Disengaged();
  } else {
    Engaged();
  }
}

void EStop::FallHandler() const {
  if (normally_closed_) {
    Engaged();
  } else {
    Disengaged();
  }
}

// ReSharper disable once CppDFAUnreachableFunctionCall
void EStop::Engaged() {
  SpiComms::Instance()->e_stop_active_ = true;
  SpiComms::SignalDataReady();
}

// ReSharper disable once CppDFAUnreachableFunctionCall
void EStop::Disengaged() { SpiComms::Instance()->e_stop_active_ = false; }
