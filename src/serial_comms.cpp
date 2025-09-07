#include "serial_comms.h"

#include "mbed_wait_api.h"

SerialComms::SerialComms() {
  serial = new mbed::UnbufferedSerial(P2_8, P2_9, 1843200);
  uint8_t buffer[10] = {'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', '\n'};
  while (true) {
    serial->write(buffer, 10);
    wait_us(1000);
  }
}