#ifndef TICKER_H
#define TICKER_H

#include <vector>

#include "LPC17xx.h"
#include "module.h"

#define NUM_TICKERS 2

class IrqTicker {
 protected:
  LPC_TIM_TypeDef* timer;
  IRQn_Type irq;
  int8_t sbit;
  uint32_t priority;

  void (*wrapper)();

 public:
  IrqTicker(LPC_TIM_TypeDef* timer, IRQn_Type irq, int8_t sbit, void (*wrapper)());

  std::vector<Module*> modules;
  uint32_t frequency;

  void configure(uint32_t frequency, uint32_t priority);

  void start();
  void handle_interrupt() const;
};

extern IrqTicker irq_tickers[NUM_TICKERS];

#endif
