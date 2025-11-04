#pragma once

#include "modules/module.h"
#include "pin.h"

class EStop final : public Module {
  bool normally_closed_ = false;

  void RiseHandler() const;
  void FallHandler() const;
  static void Engaged();
  static void Disengaged();

 public:
  explicit EStop(const Pin* pin);
};
