#include "module.h"

#include <cstdio>

#include "pin.h"

bool Module::is_stepgen() { return false; }
void Module::make_steps() {}
void Module::on_rx() {}
