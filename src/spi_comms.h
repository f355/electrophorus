#pragma once

#include "MODDMA.h"
#include "mbed.h"
#include "spi_protocol/spi_protocol.h"

class SpiComms {
  MODDMA dma_;

  MODDMA_Config rx_dma_{};
  MODDMA_Config tx_dma_{};

  MODDMA_LLI rx_lli_{};
  MODDMA_LLI tx_lli_{};

  SpiBuffer<LinuxCncState> rx_buffer_{};
  SpiBuffer<PruState> tx_buffer_{};

  void RxCallback();
  void TxCallback();
  void ErrCallback() { error("DMA error on channel %d!\n", dma_.irqProcessingChannel()); }

  SpiBuffer<LinuxCncState> linuxcnc_state_{};

  uint8_t reject_count_ = 0;
  volatile bool data_ready_ = false;
  volatile bool spi_error_ = false;

  SpiComms();

 public:
  static SpiComms* Instance();
  void Start();

  [[nodiscard]] LinuxCncState* get_linuxcnc_state() {
    static LinuxCncState empty{};
    return e_stop_active_ ? &empty : &linuxcnc_state_.AsStruct();
  }
  [[nodiscard]] volatile PruState* get_pru_state() { return &tx_buffer_.AsStruct(); }

  volatile bool e_stop_active_ = false;

  [[noreturn]] void Loop();
};
