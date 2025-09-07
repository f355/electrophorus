#ifndef E_STOP_H
#define E_STOP_H

#include "module.h"
#include "pin.h"
#include "serial_comms.h"

class EStop final : public Module {
  SerialComms* comms;

  bool normally_closed = false;

  void rise_handler() const;
  void fall_handler() const;
  void engaged() const;
  void disengaged() const;

 public:
  EStop(const Pin* pin, SerialComms* comms);
};

#endif
