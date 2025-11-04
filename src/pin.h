#pragma once

#include "LPC17xx.h"
#include "mbed.h"

class Pin {
 public:
  static LPC_GPIO_TypeDef* GetGpioPort(const uint8_t port_number) {
    static LPC_GPIO_TypeDef* gpio_ports[5] = {LPC_GPIO0, LPC_GPIO1, LPC_GPIO2, LPC_GPIO3, LPC_GPIO4};
    return gpio_ports[port_number];
  }

  Pin(unsigned char port, unsigned char pin);

  Pin* AsOutput();

  Pin* AsInput();

  bool inverting_;

  Pin* Invert();

  [[nodiscard]] bool Get() const { return inverting_ ^ ((port_->FIOPIN >> pin_) & 1); };

  void Set(const bool value) const {
    if (inverting_ ^ value)
      port_->FIOSET = 1 << pin_;
    else
      port_->FIOCLR = 1 << pin_;
  }

  [[nodiscard]] PinName ToPinName() const;

 private:
  LPC_GPIO_TypeDef* port_;

  uint8_t pin_;
  uint8_t port_number_;
};

inline void SetPullDown(const uint8_t port_number, const uint8_t pin) {
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
