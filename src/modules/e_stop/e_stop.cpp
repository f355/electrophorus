#include "e_stop.h"

eStop::eStop(volatile txData_t *tx_data, Pin *pin) : tx_data(tx_data), pin(pin->as_input()) {}

void eStop::run() {
  if (this->pin->get()) {
    tx_data->header = PRU_ESTOP;
  } else {
    tx_data->header = PRU_DATA;
  }
}
