#include "rx_listener.h"

#include "LPC17xx.h"
#include "machine_definitions.h"

RxListener::RxListener() = default;

void RxListener::start() {
  constexpr auto irq = PendSV_IRQn;
  NVIC_SetVector(irq, reinterpret_cast<uint32_t>(&RxListener::handle_interrupt));
  NVIC_EnableIRQ(irq);
}

RxListener* RxListener::instance() {
  static RxListener instance;
  return &instance;
}

void RxListener::handle_interrupt() {
  for (const auto m : instance()->modules) m->on_rx();
}

void RxListener::register_modules(const std::vector<Module*>& ms) {
  for (auto m : ms) this->modules.push_back(m);
}
