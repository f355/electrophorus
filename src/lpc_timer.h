#ifndef LPCTIMER_H
#define LPCTIMER_H

#include "LPC17xx.h"

#define NUM_TIMERS 4

class InterruptHandler {
 public:
  virtual ~InterruptHandler() = default;
  virtual void handle_interrupt() = 0;
};

class LpcTimer {
 protected:
  LPC_TIM_TypeDef* timer;
  IRQn_Type irq;
  int8_t sbit;
  uint32_t frequency;
  uint32_t priority;
  InterruptHandler* handler;

  void (*wrapper)();

 public:
  LpcTimer(LPC_TIM_TypeDef* timer, IRQn_Type irq, int8_t sbit, void (*wrapper)());

  void configure(InterruptHandler* handler, uint32_t frequency, uint32_t priority);

  void start();
  void handle_interrupt() const;
};

extern LpcTimer* lpc_timers[NUM_TIMERS];

void create_timers();

#endif
