#ifndef ESTOP_H
#define ESTOP_H

#include "module.h"
#include "pin.h"
#include "spi_comms.h"

class eStop final : public Module {
  volatile txData_t *tx_data;
  Pin *pin;

 public:
  eStop(volatile txData_t *tx_data, Pin *pin);

  void run() override;
};

#endif
