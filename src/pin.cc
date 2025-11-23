#include "pin.h"

#include "port_api.h"

LPC_GPIO_TypeDef* gpio_ports[NUM_PORTS] = {LPC_GPIO0, LPC_GPIO1, LPC_GPIO2, LPC_GPIO3, LPC_GPIO4};

Pin::Pin(const unsigned char port, const unsigned char pin) : inverting(false), pin(pin), port_number(port) {
  this->port = gpio_ports[port];
  this->port->FIOMASK &= ~(1 << this->pin);
}

PinName Pin::to_pin_name() const { return port_pin(static_cast<PortName>(port_number), pin); }
