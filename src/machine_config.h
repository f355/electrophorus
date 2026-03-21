#pragma once

#include <cstdint>
#include <vector>

#include "LPC17xx.h"
#include "PinNames.h"
#include "modules/module.h"

using std::vector;

struct SpiConfig {
  PinName mosi;
  PinName miso;
  PinName sck;
  PinName ssel;
  LPC_SSP_TypeDef* ssp;
  uint32_t dma_conn_tx;
  uint32_t dma_conn_rx;
};

void MachineInit();

vector<Module*> MachineModules();

const SpiConfig& MachineSpiConfig();

