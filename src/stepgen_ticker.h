#ifndef STEPGEN_TICKER_H
#define STEPGEN_TICKER_H

#include <vector>

#include "LPC17xx.h"
#include "machine_definitions.h"
#include "module.h"

class StepgenTicker final {
  LPC_TIM_TypeDef* timer;
  IRQn_Type irq;
  int8_t sbit;
  uint32_t frequency;
  uint32_t priority;

  std::vector<Module*> modules;

  static void irq_wrapper();

 public:
  ~StepgenTicker() = default;
  StepgenTicker();

  static StepgenTicker* instance();

  void start() const;
  void handle_interrupt() const;

  void tick() const;
  void register_modules(const std::vector<Module*>& ms);
};

#endif  // STEPGEN_TICKER_H
