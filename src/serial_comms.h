#ifndef SERIAL_COMMS_H
#define SERIAL_COMMS_H

#include "LPC17xx.h"
#include "MODDMA.h"
#include "mbed.h"
#include "protocol_definitions.h"

class SerialComms final {
  BufferedSerial* serial;

  MODDMA dma;

  MODDMA_Config* tx_dma1;
  MODDMA_Config* tx_dma2;

  volatile pruState_t tx_buf1{};
  volatile pruState_t tx_buf2{};

  linuxCncState_t rx_buf1{};
  linuxCncState_t rx_buf2{};
  // CPU-side RX assembly (staging buffer + simple scan)
  static constexpr size_t RX_FRAME_SIZE = sizeof(linuxCncState_t);
  static constexpr size_t RX_BUF_LEN = 256;
  uint8_t rx_buf[RX_BUF_LEN] = {};
  size_t rx_buf_end = 0;
  linuxCncState_t* rx_cpu_fill = &rx_buf1;

  // we start receiving/sending the first buffers - expose the second ones
  volatile linuxCncState_t* linuxcnc_state = &rx_buf2;
  volatile pruState_t* pru_state = &tx_buf2;

  void on_tx_dma_tc1();
  void on_tx_dma_tc2();

  void on_tx_dma(volatile pruState_t* our_buf, volatile pruState_t* other_buf, MODDMA_Config* other_dma);

 public:
  SerialComms();
  ~SerialComms() = default;

  // Event flags used by modules
  volatile bool e_stop_active = false;
  volatile bool data_ready = false;

  void data_ready_callback() {
    data_ready = true;
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
  }

  // Accessors for modules/ISRs
  [[nodiscard]] volatile linuxCncState_t* get_linuxcnc_state() const { return linuxcnc_state; }
  [[nodiscard]] volatile pruState_t* get_pru_state() const { return pru_state; }
  void clear_linuxcnc_state() const;

  // Stats
  volatile uint32_t tx_frames = 0;
  void poll_rx_nonblocking();

  volatile uint32_t rx_frames = 0;
  volatile uint32_t rx_crc_fail = 0;
  volatile uint32_t rx_overrun_errors = 0;  // LSR.OE occurrences
};

#endif  // SERIAL_COMMS_H
