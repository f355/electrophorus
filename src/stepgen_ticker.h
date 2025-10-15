#ifndef STEPGEN_TICKER_H
#define STEPGEN_TICKER_H

#include <vector>

#include "LPC17xx.h"
#include "modules/module.h"

class StepgenTicker final {
  LPC_TIM_TypeDef* timer = LPC_TIM0;

  std::vector<Module*> modules;

  static void tick();

  StepgenTicker() = default;

 public:
  static StepgenTicker* instance();
  void register_modules(const std::vector<Module*>& ms);
  void start() const;
};

#endif  // STEPGEN_TICKER_H
