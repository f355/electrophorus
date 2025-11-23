#pragma once

#include <vector>

#include "LPC17xx.h"
#include "modules/module.h"

class StepgenTicker final {
  LPC_TIM_TypeDef* timer_ = LPC_TIM0;

  std::vector<Module*> modules_;

  static void Tick();

  StepgenTicker() = default;

 public:
  static StepgenTicker* Instance();

  void RegisterModules(const std::vector<Module*>& ms);

  void Start() const;
};
