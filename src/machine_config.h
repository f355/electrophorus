#ifndef MACHINECONFIG_H
#define MACHINECONFIG_H

#include "module.h"
#include "spi_comms.h"

void machine_init();

vector<Module*> machine_modules(SpiComms* comms);

#endif
