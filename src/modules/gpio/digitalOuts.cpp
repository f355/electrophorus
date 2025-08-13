#include "digitalOuts.h"

#include "pin.h"

DigitalOuts::DigitalOuts(const uint8_t num_pins, const outputPin_t pins[], volatile rxData_t* rx_data)
    : outputs(&rx_data->outputs),
      num_pins(num_pins),
      ports(new LPC_GPIO_TypeDef*[num_pins]),
      pin_masks(new uint32_t[num_pins]),
      invert_mask(0x0) {
  printf("output digital pin config:\n");
  for (uint8_t i = 0; i < num_pins; i++) {
    const auto [name, port_num, pin, invert] = pins[i];
    const auto port = gpio_ports[port_num];
    port->FIOMASK &= ~(1 << pin);
    port->FIODIR |= 1 << pin;
    ports[i] = port;
    pin_masks[i] = 1 << pin;
    printf("  [%d] P%d.%d", i, port_num, pin);
    if (invert) {
      this->invert_mask |= 1 << i;
      printf("!");
    }
    printf(" - %s\n", name);
  }
}

void DigitalOuts::run() {
  const uint16_t outputs = *this->outputs ^ invert_mask;
  for (uint8_t i = 0; i < num_pins; i++) {
    if (outputs >> i & 0b1) {
      ports[i]->FIOSET |= pin_masks[i];
    } else {
      ports[i]->FIOCLR |= pin_masks[i];
    }
  }
}
