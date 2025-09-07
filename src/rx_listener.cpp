#include "rx_listener.h"

#include "LPC17xx.h"
#include "machine_definitions.h"

RxListener::RxListener() = default;

void RxListener::start() {
  NVIC_SetVector(PendSV_IRQn, reinterpret_cast<uint32_t>(&RxListener::run));
  NVIC_SetPriority(PendSV_IRQn, RX_LISTENER_PRIORITY);
  NVIC_EnableIRQ(PendSV_IRQn);
}

RxListener* RxListener::instance() {
  static RxListener instance;
  return &instance;
}

void RxListener::run() {
  for (const auto m : instance()->modules) m->on_rx();
}

void RxListener::register_modules(const std::vector<Module*>& ms) {
  for (auto m : ms) {
    if (m->listens_to_rx()) this->modules.push_back(m);
  }
}
