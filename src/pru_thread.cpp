#include "pru_thread.h"

#include "module.h"

PruThread::PruThread(const uint32_t timer_number, const uint32_t frequency, const uint32_t priority)
    : frequency(frequency) {
  this->timer = lpc_timers[timer_number];
  this->timer->configure(this, this->frequency, priority);
}

void PruThread::start() const { this->timer->start(); }

void PruThread::register_module(Module* module) { this->modules.push_back(module); }

void PruThread::handle_interrupt() {
  // iterate over the Thread pointer vector to run all instances of Module::runModule()
  for (const auto m : modules) m->run();
}
