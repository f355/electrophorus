#ifndef EPHO_RX_LISTENER_H
#define EPHO_RX_LISTENER_H

#include <vector>

#include "module.h"

class RxListener final {
  std::vector<Module*> modules;

 public:
  RxListener();

  static void start();

  static RxListener* instance();

  static void run();

  void register_modules(const std::vector<Module*>& ms);
};

#endif  // EPHO_RX_LISTENER_H
