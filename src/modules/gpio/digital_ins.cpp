#include "digital_ins.h"

#include "pin.h"

DigitalIns::DigitalIns(const uint8_t num_pins, const inputPin_t pins[], SpiComms* comms)
    : comms(comms),
      num_pins(num_pins),
      ports(new LPC_GPIO_TypeDef*[num_pins]),
      pins(new uint8_t[num_pins]),
      invert_mask(0x0) {
  printf("input digital pin config:\n");
  for (int i = 0; i < num_pins; i++) {
    const auto [name, port_num, pin, pull_down, invert] = pins[i];
    const auto port = gpio_ports[port_num];
    port->FIOMASK &= ~(1 << pin);
    port->FIODIR &= ~(1 << pin);
    ports[i] = port;
    this->pins[i] = pin;
    printf("  [%d]: P%d.%d", i, port_num, pin);
    if (invert) {
      this->invert_mask |= 1 << i;
      printf("!");
    }
    if (pull_down) {
      set_pull_down(port_num, pin);
      printf("v");
    } else {
      printf("^");
    }
    printf(" - %s\n", name);
  }
}

void DigitalIns::run_servo() {
  uint16_t pin_states = 0;
  for (uint8_t i = 0; i < num_pins; i++) pin_states |= (this->ports[i]->FIOPIN >> this->pins[i] & 0b1) << i;
  comms->get_pru_state()->inputs = pin_states ^ this->invert_mask;
}

bool DigitalIns::is_servo() { return true; }
