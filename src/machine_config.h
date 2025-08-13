#ifndef MACHINECONFIG_H
#define MACHINECONFIG_H

#include <vector>

#include "pru_thread.h"
#include "spi_comms.h"

vector<PruThread*> configure_threads(const SpiComms* comms);

#endif
