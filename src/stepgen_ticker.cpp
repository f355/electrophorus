
#include "stepgen_ticker.h"

#include "machine_definitions.h"

void StepgenTicker::start() const {
  LPC_SC->PCONP |= (1 << 1);               // power on the timer 0
  this->timer->MCR = (1 << 0) | (1 << 1);  // MR0I | MR0R: Clear TC on MR0 match and Generate Interrupt
  this->timer->PR = 0x00;
  this->timer->MR0 = SystemCoreClock / 4 / BASE_FREQUENCY;
  this->timer->TCR = (1 << 0);  // CNTEN: Start the timer

  constexpr auto irq = TIMER0_IRQn;
  NVIC_SetVector(irq, reinterpret_cast<uint32_t>(&tick));
  NVIC_EnableIRQ(irq);
}

void StepgenTicker::tick() {
  const auto t = instance();
  const unsigned int isr_mask = t->timer->IR;
  t->timer->IR = isr_mask; /* Clear the Interrupt Bit */

  for (const auto m : t->modules) m->make_steps();
}

StepgenTicker* StepgenTicker::instance() {
  static StepgenTicker instance;
  return &instance;
}

void StepgenTicker::register_modules(const std::vector<Module*>& ms) {
  for (auto m : ms) {
    if (m->is_stepgen()) this->modules.push_back(m);
  }
}
