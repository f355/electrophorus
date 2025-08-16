#include "irq_ticker.h"

#define SBIT_MR0I 0
#define SBIT_MR0R 1
#define SBIT_CNTEN 0

IrqTicker::IrqTicker(LPC_TIM_TypeDef* timer, const IRQn_Type irq, const int8_t sbit, void (*wrapper)())
    : timer(timer), irq(irq), sbit(sbit), priority(0), wrapper(wrapper), frequency(0) {};

void IrqTicker::configure(const uint32_t frequency, const uint32_t priority) {
  this->frequency = frequency;
  this->priority = priority;
};

void IrqTicker::start() {
  if (this->frequency == 0) return;

  LPC_SC->PCONP |= (1 << this->sbit);                      // power on the timer
  this->timer->MCR = (1 << SBIT_MR0I) | (1 << SBIT_MR0R);  // Clear TC on MR0 match and Generate Interrupt
  this->timer->PR = 0x00;
  this->timer->MR0 = SystemCoreClock / 4 / this->frequency;
  this->timer->TCR = (1 << SBIT_CNTEN);  // Start timer by setting the Counter Enable

  NVIC_SetVector(this->irq, reinterpret_cast<uint32_t>(this->wrapper));
  NVIC_SetPriority(this->irq, this->priority);
  NVIC_EnableIRQ(this->irq);
}

void IrqTicker::handle_interrupt() const {
  const unsigned int isr_mask = this->timer->IR;
  this->timer->IR = isr_mask; /* Clear the Interrupt Bit */

  for (const auto m : modules) m->run();
}

// need these to pass to NVIC_SetVector
void timer0_wrapper() { irq_tickers[0].handle_interrupt(); }
void timer1_wrapper() { irq_tickers[1].handle_interrupt(); }
// void timer2_wrapper() { lpc_timers[2].handle_interrupt(); }
// void timer3_wrapper() { lpc_timers[3].handle_interrupt(); }

IrqTicker irq_tickers[NUM_TICKERS] = {
    IrqTicker(LPC_TIM0, TIMER0_IRQn, 1, timer0_wrapper),  //
    IrqTicker(LPC_TIM1, TIMER1_IRQn, 2, timer1_wrapper)
    // LpcTimer(LPC_TIM2, TIMER2_IRQn, 22, timer2_wrapper),
    // LpcTimer(LPC_TIM3, TIMER3_IRQn, 23, timer3_wrapper)
};
