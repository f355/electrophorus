#ifndef MACHINECONFIG_H
#define MACHINECONFIG_H

#include "modules/module.h"
#include "spi_comms.h"

void machine_init();

vector<Module*> machine_modules(SpiComms* comms);

#endif
