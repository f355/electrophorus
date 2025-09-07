#ifndef SERIAL_COMMS_H
#define SERIAL_COMMS_H
#include "UnbufferedSerial.h"

class SerialComms {
  mbed::UnbufferedSerial *serial;

 public:
  SerialComms();
};

#endif  // SERIAL_COMMS_H
