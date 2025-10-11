#include "module.h"

#include <cstdio>

#include "pin.h"

bool Module::is_base() { return false; }
void Module::run_base() {}
void Module::on_rx() {}
