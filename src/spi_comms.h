#pragma once

#include "mbed.h"
#include "spi_protocol/spi_protocol.h"

class SpiComms {
  SpiBuffer<LinuxCncState> rx_buffer_{};
  SpiBuffer<PruState> tx_buffer_{};

  void RxCallback();
  void TxCallback();
  static void ErrCallback() { error("DMA error!\n"); }

  SpiBuffer<LinuxCncState> linuxcnc_state_{};

  volatile bool e_stop_active_ = false;

  uint8_t reject_count_ = 0;
  volatile bool data_ready_ = false;
  volatile bool spi_error_ = false;

  SpiComms();

 public:
  static SpiComms* Instance();
  static void Start();

  void EStop(bool active);

  [[nodiscard]] LinuxCncState* get_linuxcnc_state() {
    static LinuxCncState empty{};
    return e_stop_active_ ? &empty : &linuxcnc_state_.AsStruct();
  }
  [[nodiscard]] volatile PruState* get_pru_state() { return &tx_buffer_.AsStruct(); }

  [[noreturn]] void Loop();
};
