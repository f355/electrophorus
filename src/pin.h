#ifndef PIN_H
#define PIN_H

#include "LPC17xx.h"
#include "mbed.h"

#define NUM_PORTS 5

extern LPC_GPIO_TypeDef* gpio_ports[NUM_PORTS];

class Pin {
 public:
  Pin(unsigned char port, unsigned char pin);

  Pin* as_output() {
    this->port->FIODIR |= 1 << this->pin;
    return this;
  }

  Pin* as_input() {
    this->port->FIODIR &= ~(1 << this->pin);
    return this;
  }

  Pin* invert() {
    this->inverting = true;
    return this;
  };

  [[nodiscard]] bool get() const { return this->inverting ^ ((this->port->FIOPIN >> this->pin) & 1); }

  void set(const bool value) const {
    if (this->inverting ^ value)
      this->port->FIOSET = 1 << this->pin;
    else
      this->port->FIOCLR = 1 << this->pin;
  }

  [[nodiscard]] PinName to_pin_name() const;

 private:
  bool inverting;
  LPC_GPIO_TypeDef* port;

  uint8_t pin;
  uint8_t port_number;
};

inline void set_pull_down(const uint8_t port_number, const uint8_t pin) {
  if (port_number == 0 && pin < 16) {
    LPC_PINCON->PINMODE0 |= (3 << (pin * 2));
  }
  if (port_number == 0 && pin >= 16) {
    LPC_PINCON->PINMODE1 |= (3 << ((pin - 16) * 2));
  }
  if (port_number == 1 && pin < 16) {
    LPC_PINCON->PINMODE2 |= (3 << (pin * 2));
  }
  if (port_number == 1 && pin >= 16) {
    LPC_PINCON->PINMODE3 |= (3 << ((pin - 16) * 2));
  }
  if (port_number == 2 && pin < 16) {
    LPC_PINCON->PINMODE4 |= (3 << (pin * 2));
  }
  if (port_number == 3 && pin >= 16) {
    LPC_PINCON->PINMODE7 |= (3 << ((pin - 16) * 2));
  }
  if (port_number == 4 && pin >= 16) {
    LPC_PINCON->PINMODE9 |= (3 << ((pin - 16) * 2));
  }
}

#endif
