#pragma once

#include <vector>

#include "modules/module.h"

class RxListener final {
  std::vector<Module*> modules_;

 public:
  RxListener();

  static void Start();

  static RxListener* Instance();

  static void HandleInterrupt();

  void RegisterModules(const std::vector<Module*>& ms);
};
