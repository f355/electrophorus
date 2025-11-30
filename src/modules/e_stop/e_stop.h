#pragma once

#include "modules/module.h"
#include "pin.h"

class EStop final : public Module {
  static void Engaged();
  static void Disengaged();

  const Pin* pin_;
  bool initialized_;

 public:
  explicit EStop(const Pin* pin);
  void OnRx() override;
};
