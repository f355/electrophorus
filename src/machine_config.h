#ifndef MACHINECONFIG_H
#define MACHINECONFIG_H

#include "module.h"
#include "pin.h"
#include "spi_comms.h"

Pin* main_button_pin();

void machine_init();

vector<Module*> machine_base_modules(const SpiComms* comms);

vector<Module*> machine_servo_modules(SpiComms* comms);

#endif
