#pragma once

#include "mbed.h"

class SspDma {
 public:
  static SspDma* Instance();

  void ConfigureTx(const void* buffer, size_t size, const Callback<void()>& on_complete);
  void ConfigureRx(void* buffer, size_t size, const Callback<void()>& on_complete);
  void SetErrorCallback(const Callback<void()>& on_error);

  void Start();

 private:
  SspDma();

  static void IrqHandler();

  struct Lli {
    uint32_t src;
    uint32_t dst;
    uint32_t next;
    uint32_t ctrl;
  };

  Lli tx_lli_{};
  Lli rx_lli_{};
  Callback<void()> tx_cb_;
  Callback<void()> rx_cb_;
  Callback<void()> err_cb_;
};
