#include "digital_ins.h"

#include <cstdio>

#include "LPC17xx.h"
#include "pin.h"
#include "spi_comms.h"
#include "spi_protocol/machine_definitions.h"

DigitalIns::DigitalIns(const uint8_t num_pins, const inputPin_t pins[])
    : num_pins_(num_pins), ports_(new LPC_GPIO_TypeDef*[num_pins]), pins_(new uint8_t[num_pins]), invert_mask_(0x0) {
  printf("input digital pin config:\n");
  for (int i = 0; i < num_pins; i++) {
    const auto [name, port_num, pin, pull_down, invert] = pins[i];
    const auto port = Pin::GetGpioPort(port_num);
    port->FIOMASK &= ~(1 << pin);
    port->FIODIR &= ~(1 << pin);
    ports_[i] = port;
    pins_[i] = pin;
    printf("  [%d]: P%d.%d", i, port_num, pin);
    if (invert) {
      invert_mask_ |= 1 << i;
      printf("!");
    }
    if (pull_down) {
      SetPullDown(port_num, pin);
      printf("v");
    } else {
      printf("^");
    }
    printf(" - %s\n", name);
  }
}

void DigitalIns::OnRx() {
  uint16_t pin_states = 0;
  for (uint8_t i = 0; i < num_pins_; i++) pin_states |= (ports_[i]->FIOPIN >> pins_[i] & 0b1) << i;
  SpiComms::Instance()->get_pru_state()->inputs = pin_states ^ invert_mask_;
}
