#ifndef MACHINECONFIG_H
#define MACHINECONFIG_H

#include "module.h"
#include "spi_comms.h"

void machine_init();

vector<Module*> machine_base_modules(const SpiComms* comms);

vector<Module*> machine_servo_modules(const SpiComms* comms);

#endif
