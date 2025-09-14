#include "module.h"

#include <cstdio>

#include "pin.h"

bool Module::is_base() { return false; }
bool Module::is_servo() { return true; }
void Module::run_base() {}
void Module::run_servo() {}
