#ifndef COMMS_H
#define COMMS_H

#include "MODDMA.h"
#include "mbed.h"
#include "spi_protocol.h"

class SpiComms {
  MODDMA dma;

  MODDMA_Config* rx_dma1;
  MODDMA_Config* rx_dma2;
  MODDMA_Config* tx_dma1;
  MODDMA_Config* tx_dma2;

  linuxCncState_t temp_rx_buffer1{};
  linuxCncState_t temp_rx_buffer2{};

  volatile linuxCncState_t* linuxcnc_state = &temp_rx_buffer2;
  volatile pruState_t pru_state{};

  uint8_t reject_count = 0;
  volatile bool data_ready = false;
  volatile bool spi_error = false;

  void tx1_callback();
  void tx2_callback();
  void rx1_callback();
  void rx2_callback();
  void err_callback();

  void rx_callback_impl(const linuxCncState_t& rx_buffer, MODDMA_Config* other_rx);

 public:
  SpiComms();

  static void data_ready_callback();

  [[nodiscard]] volatile linuxCncState_t* get_linuxcnc_state() const;
  [[nodiscard]] volatile pruState_t* get_pru_state();

  volatile bool e_stop_active = false;

  [[noreturn]] void loop();
};

#endif
