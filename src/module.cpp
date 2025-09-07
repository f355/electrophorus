#include "module.h"

#include <cstdio>

#include "pin.h"

bool Module::listens_to_rx() { return false; }
bool Module::is_stepgen() { return false; }
void Module::make_steps() {}
void Module::on_rx() {}
