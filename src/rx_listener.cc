#include "rx_listener.h"

#include "LPC17xx.h"

RxListener::RxListener() = default;

void RxListener::Start() {
  constexpr auto irq = PendSV_IRQn;
  NVIC_SetVector(irq, reinterpret_cast<uint32_t>(&RxListener::HandleInterrupt));
  NVIC_EnableIRQ(irq);
}

RxListener* RxListener::Instance() {
  static RxListener instance;
  return &instance;
}

void RxListener::HandleInterrupt() {
  for (const auto m : Instance()->modules_) m->OnRx();
}

void RxListener::RegisterModules(const std::vector<Module*>& ms) {
  for (auto m : ms) modules_.push_back(m);
}
