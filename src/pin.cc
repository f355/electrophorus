#include "pin.h"

#include "port_api.h"

Pin::Pin(const unsigned char port, const unsigned char pin) : inverting_(false), pin_(pin), port_number_(port) {
  port_ = GetGpioPort(port);
  port_->FIOMASK &= ~(1 << pin_);
}

Pin* Pin::AsOutput() {
  port_->FIODIR |= 1 << pin_;
  return this;
}

Pin* Pin::AsInput() {
  port_->FIODIR &= ~(1 << pin_);
  return this;
}

Pin* Pin::Invert() {
  inverting_ = true;
  return this;
}

PinName Pin::ToPinName() const { return port_pin(static_cast<PortName>(port_number_), pin_); }
