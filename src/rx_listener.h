#pragma once

#include <vector>

#include "LPC17xx.h"
#include "modules/module.h"

class RxListener final {
  std::vector<Module*> modules_;

 public:
  RxListener();

  static void Start();

  static RxListener* Instance();

  void RegisterModules(const std::vector<Module*>& ms);

  static void HandleRx();

  static void HandleRxDeferred() {
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;  // trigger PendSV IRQ to signal that the data is ready
  }
};
