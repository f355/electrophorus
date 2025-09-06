#include "module.h"

#include <cstdio>

#include "pin.h"

bool Module::is_servo() { return true; }
bool Module::is_base() { return false; }

void Module::run_servo() {}
void Module::run_base() {}
