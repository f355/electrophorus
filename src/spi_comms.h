#pragma once

#include "mbed.h"
#include "spi_protocol/spi_protocol.h"

class SpiComms {
  struct DmaLli {
    uint32_t src;
    uint32_t dst;
    uint32_t next;
    uint32_t ctrl;
  };

  DmaLli tx_lli_{};
  DmaLli rx_lli_{};

  SpiBuffer<PruState> tx_buffer_{};
  SpiBuffer<LinuxCncState> rx_buffer_{};

  void TxCallback();
  void RxCallback();
  static void ErrCallback() { error("DMA error!\n"); }
  static void DmaIrqHandler();

  [[nodiscard]] uint32_t CtrlWord(bool src_inc, bool dst_inc) const;
  static uint32_t ConfigWord(uint32_t transfer_type, uint32_t conn);

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
