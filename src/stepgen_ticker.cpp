#include "stepgen_ticker.h"

#include "machine_definitions.h"

StepgenTicker::StepgenTicker() = default;

StepgenTicker* StepgenTicker::instance() {
  static StepgenTicker instance;
  return &instance;
}

void StepgenTicker::register_modules(const std::vector<Module*>& ms) {
  for (auto m : ms) {
    if (m->is_stepgen()) this->modules.push_back(m);
  }
}

void StepgenTicker::start() const {
  LPC_SC->PCONP |= 1 << SBIT_EN;                       // power on the timer
  this->timer->MCR = 1 << SBIT_MR0I | 1 << SBIT_MR0R;  // Clear TC on MR0 match and Generate Interrupt
  this->timer->PR = 0x00;
  this->timer->MR0 = SystemCoreClock / 4 / BASE_FREQUENCY;
  this->timer->TCR = 1 << SBIT_CNTEN;  // Start the timer by setting the Counter Enable

  constexpr auto irq = TIMER0_IRQn;
  NVIC_SetVector(irq, reinterpret_cast<uint32_t>(&StepgenTicker::irq_wrapper));
  NVIC_SetPriority(irq, BASE_TICKER_PRIORITY);
  NVIC_EnableIRQ(irq);
}

// ReSharper disable once CppDFAUnreachableFunctionCall
void StepgenTicker::handle_interrupt() const {
  const auto isr_mask = this->timer->IR;
  this->timer->IR = isr_mask; /* Clear the Interrupt Bit */

  for (const auto m : this->modules) m->make_steps();
}

void StepgenTicker::irq_wrapper() { instance()->handle_interrupt(); }
