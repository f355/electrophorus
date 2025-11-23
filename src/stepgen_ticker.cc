#include "stepgen_ticker.h"

#include <vector>

#include "LPC17xx.h"
#include "machine_definitions.h"
#include "modules/module.h"

void StepgenTicker::Start() const {
  LPC_SC->PCONP |= (1 << 1);                // power on the timer 0
  timer_->MCR = (1 << 0) | (1 << 1);  // MR0I | MR0R: Clear TC on MR0 match and Generate Interrupt
  timer_->PR = 0x00;
  timer_->MR0 = SystemCoreClock / 4 / kStepgenTickFrequency;
  timer_->TCR = (1 << 0);  // CNTEN: Start the timer

  constexpr auto irq = TIMER0_IRQn;
  NVIC_SetVector(irq, reinterpret_cast<uint32_t>(&Tick));
  NVIC_EnableIRQ(irq);
}

void StepgenTicker::Tick() {
  const auto t = Instance();
  const unsigned int isr_mask = t->timer_->IR;
  t->timer_->IR = isr_mask; /* Clear the Interrupt Bit */

  for (const auto m : t->modules_) m->MakeSteps();
}

StepgenTicker* StepgenTicker::Instance() {
  static StepgenTicker instance;
  return &instance;
}

void StepgenTicker::RegisterModules(const std::vector<Module*>& ms) {
  for (auto m : ms)
    if (m->IsStepgen()) modules_.push_back(m);
}
