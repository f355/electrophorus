#include "stepgen_ticker.h"

static constexpr uint32_t SBIT_MR0I = 0;
static constexpr uint32_t SBIT_MR0R = 1;
static constexpr uint32_t SBIT_CNTEN = 0;

StepgenTicker::StepgenTicker()
    : timer(LPC_TIM0), irq(TIMER0_IRQn), sbit(1), frequency(STEPGEN_FREQUENCY), priority(STEPGEN_TICKER_PRIORITY) {}

void StepgenTicker::start() const {
  LPC_SC->PCONP |= (1 << this->sbit);                      // power on the timer
  this->timer->MCR = (1 << SBIT_MR0I) | (1 << SBIT_MR0R);  // Clear TC on MR0 match and Generate Interrupt
  this->timer->PR = 0x00;
  this->timer->MR0 = SystemCoreClock / 4 / this->frequency;
  this->timer->TCR = (1 << SBIT_CNTEN);  // Start the timer by setting the Counter Enable

  NVIC_SetVector(this->irq, reinterpret_cast<uint32_t>(&StepgenTicker::irq_wrapper));
  NVIC_SetPriority(this->irq, this->priority);
  NVIC_EnableIRQ(this->irq);
}

void StepgenTicker::handle_interrupt() const {
  const unsigned int isr_mask = this->timer->IR;
  this->timer->IR = isr_mask; /* Clear the Interrupt Bit */

  this->tick();
}

void StepgenTicker::irq_wrapper() { instance()->handle_interrupt(); }

StepgenTicker* StepgenTicker::instance() {
  static StepgenTicker instance;
  return &instance;
}

void StepgenTicker::tick() const {
  for (const auto m : this->modules) m->make_steps();
}

void StepgenTicker::register_modules(const std::vector<Module*>& ms) {
  for (auto m : ms) {
    if (m->is_stepgen()) this->modules.push_back(m);
  }
}
