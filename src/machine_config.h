#ifndef MACHINECONFIG_H
#define MACHINECONFIG_H

#include "module.h"
#include "serial_comms.h"

void machine_init();

vector<Module*> machine_modules(SerialComms* comms);

#endif
