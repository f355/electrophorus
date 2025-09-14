#include "module.h"

#include <cstdio>

#include "pin.h"

bool Module::listens_to_rx() { return false; }
bool Module::is_base() { return false; }
bool Module::is_servo() { return false; }
void Module::run_base() {}
void Module::run_servo() {}
void Module::on_rx() {}
