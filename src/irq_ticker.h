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
  uint32_t frequency;
  uint32_t priority;

  std::vector<Module*> modules;

  void (*wrapper)();

 public:
  virtual ~IrqTicker() = default;
  IrqTicker(LPC_TIM_TypeDef* timer, IRQn_Type irq, int8_t sbit, uint32_t frequency, uint32_t priority,
            void (*wrapper)());

  void start();
  void handle_interrupt() const;
  virtual void tick() const = 0;
  virtual void register_modules(const std::vector<Module*>& ms) = 0;
};

class BaseTicker final : public IrqTicker {
  static void irq_wrapper();

 public:
  BaseTicker();
  static BaseTicker* instance();
  void tick() const override;
  void register_modules(const std::vector<Module*>& ms) override;
};

class ServoTicker final : public IrqTicker {
  static void irq_wrapper();

 public:
  ServoTicker();
  static ServoTicker* instance();
  void tick() const override;
  void register_modules(const std::vector<Module*>& ms) override;
};

#endif  // IRQ_TICKER_H
