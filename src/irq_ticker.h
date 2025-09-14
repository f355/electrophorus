#ifndef IRQ_TICKER_H
#define IRQ_TICKER_H

#include <vector>

#include "LPC17xx.h"
#include "module.h"

class IrqTicker {
 protected:
  LPC_TIM_TypeDef* timer;
  IRQn_Type irq;
  int8_t sbit;
  uint32_t priority;

  void (*wrapper)();

 public:
  virtual ~IrqTicker() = default;
  IrqTicker(LPC_TIM_TypeDef* timer, IRQn_Type irq, int8_t sbit, uint32_t frequency, uint32_t priority,
            void (*wrapper)());

  uint32_t frequency;
  std::vector<Module*> modules;

  void start();
  void handle_interrupt() const;
};

class BaseTicker final : public IrqTicker {
  static void irq_wrapper();

 public:
  BaseTicker();
  static BaseTicker* instance();
};

class ServoTicker final : public IrqTicker {
  static void irq_wrapper();

 public:
  ServoTicker();
  static ServoTicker* instance();
};

#endif  // IRQ_TICKER_H
