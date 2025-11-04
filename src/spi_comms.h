#ifndef COMMS_H
#define COMMS_H

#include "MODDMA.h"
#include "mbed.h"
#include "spi_protocol.h"

class SpiComms {
  MODDMA dma;

  MODDMA_Config* rx_dma1;
  MODDMA_Config* rx_dma2;
  MODDMA_Config* tx_dma;
  MODDMA_LLI tx_lli;

  SpiBuffer<LinuxCncState> rx_buffer1{};
  SpiBuffer<LinuxCncState> rx_buffer2{};

  volatile LinuxCncState* linuxcnc_state = &rx_buffer2.state();

  SpiBuffer<PruState> tx_buffer{};

  uint8_t reject_count = 0;
  volatile bool data_ready = false;
  volatile bool spi_error = false;

  void rx1_callback();
  void rx2_callback();
  void err_callback();

  void rx_callback_impl(const LinuxCncState& rx_buffer, MODDMA_Config* other_rx);

 public:
  SpiComms();

  static void data_ready_callback();

  [[nodiscard]] volatile LinuxCncState* get_linuxcnc_state() const;
  [[nodiscard]] volatile PruState* get_pru_state();

  volatile bool e_stop_active = false;

  [[noreturn]] void loop();
};

#endif
