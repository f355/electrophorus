#ifndef COMMS_H
#define COMMS_H

#include "MODDMA.h"
#include "mbed.h"
#include "spi_data.h"

class SpiComms {
  MODDMA dma;

  MODDMA_Config* cmd_dma;
  MODDMA_Config* rx_dma;
  MODDMA_Config* tx_dma;
  MODDMA_Config* tx_cmd_dma;


  linuxCncState_t volatile linuxcnc_state1{};
  linuxCncState_t volatile linuxcnc_state2{};
  pruState_t volatile pru_state1{};
  pruState_t volatile pru_state2{};

  volatile uint32_t rx_cmd = 0;
  volatile uint32_t tx_cmd = PRU_DATA;

  // Discard buffer for header-rejected payloads; sized to the larger of the two payload structs
  static constexpr size_t RX_DISCARD_SIZE = (
      sizeof(linuxCncState_t) > sizeof(pruState_t) ? sizeof(linuxCncState_t) : sizeof(pruState_t));
  volatile uint8_t rx_discard[RX_DISCARD_SIZE]{};

  // Dedicated zero TX buffer for WRITE payloads (size = LinuxCNC payload)
  static constexpr size_t WRITE_TX_SIZE = sizeof(linuxCncState_t);
  volatile uint8_t tx_zero[WRITE_TX_SIZE]{};

  // Mode of the last completed or currently armed payload
  enum class RxMode : uint8_t { None = 0, Read = 1, Write = 2, Discard = 3 };
  volatile RxMode rx_mode = RxMode::None;

  uint8_t reject_count = 0;
  volatile bool data_ready = false;
  volatile bool spi_error = false;

  void cmd_callback();
  void rx_callback();

  // Double-buffer pointers
  volatile linuxCncState_t* linuxcnc_back = nullptr;
  volatile pruState_t* pru_back = nullptr;

 public:
  SpiComms();

  static void data_ready_callback();

  linuxCncState_t volatile* linuxcnc_state;
  pruState_t volatile* pru_state;

  volatile bool e_stop_active = false;

  [[noreturn]] void loop();

  [[nodiscard]] linuxCncState_t volatile* get_linuxcnc_state() const;
  [[nodiscard]] pruState_t volatile* get_pru_state() const;
};

#endif
