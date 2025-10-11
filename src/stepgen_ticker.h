#ifndef STEPGEN_TICKER_H
#define STEPGEN_TICKER_H

#include <vector>

#include "LPC17xx.h"
#include "module.h"

class StepgenTicker final {
  static constexpr uint32_t SBIT_EN = 1;
  static constexpr uint32_t SBIT_MR0I = 0;
  static constexpr uint32_t SBIT_MR0R = 1;
  static constexpr uint32_t SBIT_CNTEN = 0;

  LPC_TIM_TypeDef* timer = LPC_TIM0;

  std::vector<Module*> modules;

  static void irq_wrapper();
  void handle_interrupt() const;

 public:
  StepgenTicker();
  static StepgenTicker* instance();

  void register_modules(const std::vector<Module*>& ms);
  void start() const;
};

#endif  // STEPGEN_TICKER_H
