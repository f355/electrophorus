#include "irq_ticker.h"

#include "machine_definitions.h"

#define SBIT_MR0I 0
#define SBIT_MR0R 1
#define SBIT_CNTEN 0

IrqTicker::IrqTicker(LPC_TIM_TypeDef* timer, const IRQn_Type irq, const int8_t sbit, const uint32_t frequency,
                     const uint32_t priority, void (*wrapper)())
    : timer(timer), irq(irq), sbit(sbit), frequency(frequency), priority(priority), wrapper(wrapper) {};

void IrqTicker::start() {
  if (this->frequency == 0) return;

  LPC_SC->PCONP |= (1 << this->sbit);                      // power on the timer
  this->timer->MCR = (1 << SBIT_MR0I) | (1 << SBIT_MR0R);  // Clear TC on MR0 match and Generate Interrupt
  this->timer->PR = 0x00;
  this->timer->MR0 = SystemCoreClock / 4 / this->frequency;
  this->timer->TCR = (1 << SBIT_CNTEN);  // Start the timer by setting the Counter Enable

  NVIC_SetVector(this->irq, reinterpret_cast<uint32_t>(this->wrapper));
  NVIC_SetPriority(this->irq, this->priority);
  NVIC_EnableIRQ(this->irq);
}

void IrqTicker::handle_interrupt() const {
  const unsigned int isr_mask = this->timer->IR;
  this->timer->IR = isr_mask; /* Clear the Interrupt Bit */

  this->tick();
}

BaseTicker::BaseTicker() : IrqTicker(LPC_TIM0, TIMER0_IRQn, 1, BASE_FREQUENCY, BASE_TICKER_PRIORITY, irq_wrapper) {}

void BaseTicker::irq_wrapper() { instance()->handle_interrupt(); }

BaseTicker* BaseTicker::instance() {
  static BaseTicker instance;
  return &instance;
}

void BaseTicker::tick() const {
  for (const auto m : this->modules) m->run_base();
}

void BaseTicker::register_modules(const std::vector<Module*>& ms) {
  for (auto m : ms) {
    if (m->is_base()) this->modules.push_back(m);
  }
}

ServoTicker::ServoTicker() : IrqTicker(LPC_TIM1, TIMER1_IRQn, 2, SERVO_FREQUENCY, SERVO_TICKER_PRIORITY, irq_wrapper) {}

void ServoTicker::irq_wrapper() { instance()->handle_interrupt(); }

ServoTicker* ServoTicker::instance() {
  static ServoTicker instance;
  return &instance;
}

void ServoTicker::tick() const {
  for (const auto m : this->modules) m->run_servo();
}

void ServoTicker::register_modules(const std::vector<Module*>& ms) {
  for (auto m : ms) {
    if (m->is_servo()) this->modules.push_back(m);
  }
}