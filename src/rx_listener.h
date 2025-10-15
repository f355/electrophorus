#ifndef EPHO_RX_LISTENER_H
#define EPHO_RX_LISTENER_H

#include <vector>

#include "modules/module.h"

class RxListener final {
  std::vector<Module*> modules;

 public:
  RxListener();

  static void start();

  static RxListener* instance();

  static void handle_interrupt();

  void register_modules(const std::vector<Module*>& ms);
};

#endif  // EPHO_RX_LISTENER_H
