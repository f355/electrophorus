#ifndef PRUTHREAD_H
#define PRUTHREAD_H

#include <vector>

#include "LPC17xx.h"
#include "lpc_timer.h"

using namespace std;

class Module;

class PruThread final : public InterruptHandler {
  LpcTimer *timer;

  vector<Module *> modules;  // vector containing pointers to Thread modules

 public:
  PruThread(uint32_t timer_number, uint32_t frequency, uint32_t priority);

  uint32_t frequency;

  void register_module(Module *module);
  void start() const;
  void handle_interrupt() override;
};

#endif
