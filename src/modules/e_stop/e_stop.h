#pragma once

#include "modules/module.h"
#include "pin.h"

class EStop final : public Module {
  static void Engaged();
  static void Disengaged();

 public:
  explicit EStop(const Pin* pin);
};
