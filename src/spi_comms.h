#ifndef COMMS_H
#define COMMS_H

#include "mbed.h"
#include "spi_protocol.h"

class SpiComms {
  // Double buffers
  linuxCncState_t linuxcnc_state1{};
  linuxCncState_t linuxcnc_state2{};
  pruState_t pru_state1{};
  pruState_t pru_state2{};

  // Published pointers and back buffers
  volatile linuxCncState_t* linuxcnc_back = nullptr;
  volatile pruState_t* pru_back = nullptr;

  // Current command and transfer state
  enum class PruCommand : uint8_t { None = 0, Read = 1, Write = 2, Invalid = 3 };
  volatile PruCommand current_cmd = PruCommand::None;

  uint32_t cmd_word = 0;

  // Payload bookkeeping
  size_t bytes_received = 0;
  size_t bytes_transmitted = 0;
  uint8_t* rx_ptr = nullptr;
  const uint8_t* tx_ptr = nullptr;

  size_t discard_size = 0;  // bytes to discard on bad command; can be the size of either struct

  // Streaming CRC state
  uint32_t crc = 0;  // running CRC (TX or RX); holds finalized CRC in TX tail

  uint8_t reject_count = 0;
  volatile bool data_ready = false;
  volatile bool spi_error = false;

  // mbed SPI slave interface
  static constexpr PinName SPI_MOSI = P0_18;
  static constexpr PinName SPI_MISO = P0_17;
  static constexpr PinName SPI_SCK = P0_15;
  static constexpr PinName SPI_SSEL = P0_16;
  spi_t spi{};

  [[gnu::always_inline]] [[nodiscard]] bool spi_readable() const { return spi.spi->SR & (1u << 2); }  // RNE
  [[gnu::always_inline]] [[nodiscard]] uint8_t spi_read() const { return spi.spi->DR; }
  [[gnu::always_inline]] [[nodiscard]] bool spi_writeable() const { return spi.spi->SR & (1u << 1); }  // TNF
  [[gnu::always_inline]] void spi_write(const uint8_t value) const { spi.spi->DR = value; }

  void spi_rx_irq(const bool enabled) const {
    constexpr uint32_t mask = (1u << 2) | (1u << 0);  // RXIM | RORIM
    if (enabled) {
      spi.spi->IMSC |= mask;
    } else {
      spi.spi->IMSC &= ~mask;
    }
  }
  void spi_tx_irq(const bool enabled) const {
    constexpr uint32_t mask = 1u << 3;  // TXIM
    if (enabled) {
      spi.spi->IMSC |= mask;
    } else {
      spi.spi->IMSC &= ~mask;
    }
  }

  inline void preload_cmd_response() const;
  inline void transmit_read_response();
  inline void receive_write_payload();
  inline void discard_payload();
  inline void wait_for_command();

  // Helper to consume one RX byte during WRITE (updates buffer and CRC)
  inline void try_read_payload_byte();

 public:
  SpiComms();

  static SpiComms* s_instance;

  void ssp0_irq();

  static void data_ready_callback();

  linuxCncState_t volatile* linuxcnc_state;
  pruState_t volatile* pru_state;

  volatile bool e_stop_active = false;

  [[noreturn]] void loop();

  [[nodiscard]] linuxCncState_t volatile* get_linuxcnc_state() const;
  [[nodiscard]] pruState_t volatile* get_pru_state() const;
};

#endif
