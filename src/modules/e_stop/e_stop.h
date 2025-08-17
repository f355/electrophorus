#ifndef E_STOP_H
#define E_STOP_H

#include "module.h"
#include "pin.h"
#include "spi_comms.h"

class EStop final : public Module {
  SpiComms* comms;

  void rise_handler() const;
  void fall_handler() const;

 public:
  EStop(const Pin* pin, SpiComms* comms);
};

#endif
