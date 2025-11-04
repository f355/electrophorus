#include "digital_outs.h"

#include "pin.h"

DigitalOuts::DigitalOuts(const uint8_t num_pins, const outputPin_t pins[])
    : num_pins_(num_pins),
      ports_(new LPC_GPIO_TypeDef*[num_pins]),
      pin_masks_(new uint32_t[num_pins]),
      invert_mask_(0x0) {
  printf("output digital pin config:\n");
  for (uint8_t i = 0; i < num_pins; i++) {
    const auto [name, port_num, pin, invert] = pins[i];
    const auto port = Pin::GetGpioPort(port_num);
    port->FIOMASK &= ~(1 << pin);
    port->FIODIR |= 1 << pin;
    ports_[i] = port;
    pin_masks_[i] = 1 << pin;
    printf("  [%d] P%d.%d", i, port_num, pin);
    if (invert) {
      invert_mask_ |= 1 << i;
      printf("!");
    }
    printf(" - %s\n", name);
  }
}

void DigitalOuts::OnRx() {
  auto outputs = SpiComms::Instance()->get_linuxcnc_state()->outputs;
  if (outputs == last_outputs_) return;
  last_outputs_ = outputs;
  outputs ^= invert_mask_;
  for (uint8_t i = 0; i < num_pins_; i++) {
    if (outputs >> i & 0b1) {
      ports_[i]->FIOSET |= pin_masks_[i];
    } else {
      ports_[i]->FIOCLR |= pin_masks_[i];
    }
  }
}
