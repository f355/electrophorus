#pragma once

#include "MODDMA.h"
#include "mbed.h"
#include "spi_protocol/spi_protocol.h"

class SpiComms {
  MODDMA dma_;

  MODDMA_Config rx_dma1_{};
  MODDMA_Config rx_dma2_{};
  MODDMA_Config tx_dma_{};
  MODDMA_LLI tx_lli_{};

  SpiBuffer<LinuxCncState> rx_buffer1_{};
  SpiBuffer<LinuxCncState> rx_buffer2_{};

  volatile LinuxCncState* linuxcnc_state = &rx_buffer2_.AsStruct();

  SpiBuffer<PruState> tx_buffer_{};

  uint8_t reject_count_ = 0;
  volatile bool data_ready_ = false;
  volatile bool spi_error_ = false;

  void Rx1Callback() { RxCallbackImpl(rx_buffer1_.AsStruct(), rx_dma2_); }
  void Rx2Callback() { RxCallbackImpl(rx_buffer2_.AsStruct(), rx_dma1_); }
  void ErrCallback() { error("DMA error on channel %d!\n", dma_.irqProcessingChannel()); }

  void RxCallbackImpl(const LinuxCncState& rx_buffer, MODDMA_Config& other_rx);

  SpiComms();

 public:
  static SpiComms* Instance();
  void Start();

  [[nodiscard]] volatile LinuxCncState* get_linuxcnc_state() const {
    static LinuxCncState empty{};
    return e_stop_active_ ? &empty : linuxcnc_state;
  }
  [[nodiscard]] volatile PruState* get_pru_state() { return &tx_buffer_.AsStruct(); }

  volatile bool e_stop_active_ = false;

  [[noreturn]] void Loop();
};
