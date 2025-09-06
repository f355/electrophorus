#ifndef MACHINECONFIG_H
#define MACHINECONFIG_H

#include "module.h"
#include "spi_comms.h"

void machine_init();

vector<Module*> stepper_modules(const SpiComms* comms);

vector<Module*> slow_modules(SpiComms* comms);

#endif
