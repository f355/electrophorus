#include "spi_comms.h"

#define SPI_MOSI P0_18
#define SPI_MISO P0_17
#define SPI_SCK P0_15
#define SPI_SSEL P0_16

#define MAX_SPI_DELAY 5  // maximum number of (roughly) milliseconds without communication from LinuxCNC

enum State { ST_IDLE = 0, ST_RUNNING, ST_RESET };

// ReSharper disable once CppDFAUnreachableFunctionCall
static uint32_t crc32_ieee(const volatile uint8_t* data, const size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    const uint8_t byte = data[i];
    crc ^= byte;
    for (int k = 0; k < 8; ++k) {
      const uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

SpiComms::SpiComms() : cmd_dma(new MODDMA_Config()), rx_dma(new MODDMA_Config()), tx_dma(new MODDMA_Config()), tx_cmd_dma(new MODDMA_Config()) {
  // Initialize buffer pointers
  this->linuxcnc_state = &this->linuxcnc_state1;
  this->linuxcnc_back = &this->linuxcnc_state2;
  this->pru_state = &this->pru_state1;
  this->pru_back = &this->pru_state2;

  // Command word sent in the header phase
  this->tx_cmd = PRU_DATA;
  // Initial CRC so the very first PRU_DATA payload is valid
  this->pru_state1.crc32 =
      crc32_ieee(reinterpret_cast<const volatile uint8_t*>(&this->pru_state1), offsetof(pruState_t, crc32));

  // just initialize the peripheral, the communication is done through DMA
  new SPISlave(SPI_MOSI, SPI_MISO, SPI_SCK, SPI_SSEL);

  // Base config for payload TX (no src/size yet)
  tx_dma->channelNum(MODDMA::Channel_0)
      ->transferType(MODDMA::m2p)
      ->dstConn(MODDMA::SSP0_Tx);

  // Dedicated TX channel for 4-byte command header
  tx_cmd_dma->channelNum(MODDMA::Channel_1)
      ->transferType(MODDMA::m2p)
      ->dstConn(MODDMA::SSP0_Tx)
      ->srcMemAddr(reinterpret_cast<uint32_t>(&tx_cmd))
      ->transferSize(sizeof(tx_cmd));

  cmd_dma->channelNum(MODDMA::Channel_2)
      ->transferType(MODDMA::p2m)
      ->srcConn(MODDMA::SSP0_Rx)
      ->attach_tc(this, &SpiComms::cmd_callback)
      ->dstMemAddr(reinterpret_cast<uint32_t>(&rx_cmd))
      ->transferSize(sizeof(rx_cmd));

  rx_dma->channelNum(MODDMA::Channel_3)
      ->transferType(MODDMA::p2m)
      ->srcConn(MODDMA::SSP0_Rx)
      ->attach_tc(this, &SpiComms::rx_callback);

  NVIC_SetPriority(DMA_IRQn, DMA_PRIORITY);

  // Enable SSP0 for DMA
  LPC_SSP0->DMACR = 0;
  LPC_SSP0->DMACR = 1 << 1 | 1 << 0;  // TX,RX DMA Enable

  // Pass the configurations to the controller
  dma.Prepare(cmd_dma);
  dma.Prepare(tx_cmd_dma);
}

void SpiComms::cmd_callback() {
  dma.Disable(dma.irqProcessingChannel());
  dma.clearTcIrq();

  data_ready = false;
  spi_error = false;

  switch (this->rx_cmd) {
    case PRU_READ: {
      reject_count = 0;
      data_ready = true;
      // Transmit the back buffer (stable snapshot)
      this->pru_back->crc32 =
          crc32_ieee(reinterpret_cast<const volatile uint8_t*>(this->pru_back), offsetof(pruState_t, crc32));
      tx_dma->srcMemAddr(reinterpret_cast<uint32_t>(this->pru_back))->transferSize(sizeof(pruState_t));
      dma.Prepare(tx_dma);

      // Discard incoming PRU payload bytes for READ
      rx_dma->dstMemAddr(reinterpret_cast<uint32_t>(this->rx_discard))->transferSize(sizeof(pruState_t));
      dma.Prepare(rx_dma);

      rx_mode = RxMode::Read;  // READ payload in progress
      break;
    }
    case PRU_WRITE: {
      reject_count = 0;
      data_ready = true;  // keep comms alive even if we skip updating under e-stop

      // Arm TX to send constant zeros for the WRITE payload
      tx_dma->srcMemAddr(reinterpret_cast<uint32_t>(this->tx_zero))->transferSize(sizeof(linuxCncState_t));
      dma.Prepare(tx_dma);

      // Arm RX to write incoming LinuxCNC payload into the back buffer
      rx_dma->dstMemAddr(reinterpret_cast<uint32_t>(this->linuxcnc_back))->transferSize(sizeof(linuxCncState_t));
      dma.Prepare(rx_dma);

      rx_mode = RxMode::Write;  // WRITE payload in progress
      break;
    }
    default: {
      reject_count++;
      if (reject_count > 5) spi_error = true;
      // Unknown/corrupt command: LinuxCNC may still clock a payload.
      const size_t discard_size = (rx_mode == RxMode::Read) ? sizeof(linuxCncState_t) : sizeof(pruState_t);
      rx_mode = RxMode::Discard;  // mark this as discard-only
      tx_cmd_dma->srcMemAddr(reinterpret_cast<uint32_t>(&this->tx_cmd))->transferSize(sizeof(tx_cmd));
      dma.Prepare(tx_cmd_dma);
      rx_dma->dstMemAddr(reinterpret_cast<uint32_t>(this->rx_discard))->transferSize(discard_size);
      dma.Prepare(rx_dma);
    }
  }
}

void SpiComms::rx_callback() {
  dma.Disable(dma.irqProcessingChannel());
  dma.clearTcIrq();

  tx_cmd_dma->srcMemAddr(reinterpret_cast<uint32_t>(&tx_cmd))->transferSize(sizeof(tx_cmd));
  dma.Prepare(tx_cmd_dma);

  switch (rx_mode) {
    case RxMode::Read: {
      // READ payload completed; swap writer and back buffers
      volatile pruState_t* tmp = this->pru_state;
      this->pru_state = this->pru_back;
      this->pru_back = tmp;
      break;
    }
    case RxMode::Write: {
      // WRITE payload completed; validate and publish
      if (!this->e_stop_active) {
        volatile linuxCncState_t* buf = this->linuxcnc_back;
        if (buf->crc32 ==
            crc32_ieee(reinterpret_cast<const volatile uint8_t*>(buf), offsetof(linuxCncState_t, crc32))) {
          // swap: published <-> back
          volatile linuxCncState_t* tmp = this->linuxcnc_state;
          this->linuxcnc_state = buf;
          this->linuxcnc_back = tmp;
          data_ready_callback();
        } else {
          reject_count++;
          if (reject_count > 5) spi_error = true;
        }
      }
      break;
    }
    default:;
  }

  // Re-arm for next command header
  cmd_dma->dstMemAddr(reinterpret_cast<uint32_t>(&rx_cmd))->transferSize(sizeof(rx_cmd));
  dma.Prepare(cmd_dma);

  // Reset transient markers
}

void SpiComms::data_ready_callback() {
  // trigger PendSV IRQ to signal that the data is ready
  SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

void SpiComms::loop() {
  uint8_t spi_delay = 0;
  State current_state = ST_IDLE;
  State prev_state = ST_RESET;
  bool prev_e_stop_active = this->e_stop_active;

  while (true) {
    Watchdog::get_instance().kick();

    if (this->e_stop_active != prev_e_stop_active) {
      if (this->e_stop_active) {
        printf("e-stop pressed, machine halted!\n");
      } else {
        printf("e-stop released, resuming operation...\n");
      }
      prev_e_stop_active = this->e_stop_active;
    }

    if (this->spi_error) {
      printf("SPI communication error, ignoring. If you see a lot of these, check your cabling.\n");
      this->spi_error = false;
    }

    switch (current_state) {
      case ST_IDLE:
        if (current_state != prev_state) {
          printf("waiting for LinuxCNC...\n");
        }
        prev_state = current_state;

        if (this->data_ready) {
          current_state = ST_RUNNING;
        }
        break;

      case ST_RUNNING:
        if (current_state != prev_state) {
          printf("running...\n");
        }
        prev_state = current_state;

        if (this->data_ready) {
          spi_delay = 0;
          this->data_ready = false;
        } else {
          spi_delay++;
        }

        if (spi_delay > MAX_SPI_DELAY) {
          printf("no communication from LinuxCNC, e-stop active?\n");
          spi_delay = 0;
          current_state = ST_RESET;
        }
        break;
      case ST_RESET:
        if (current_state != prev_state) {
          printf("resetting receive buffer...\n");
        }
        prev_state = current_state;

        // clear the whole LinuxCNC state buffer
        // can't memset volatile memory, so use a loop instead
        for (size_t i = 0; i < sizeof(linuxCncState_t); ++i)
          reinterpret_cast<volatile uint8_t*>(this->linuxcnc_state)[i] = 0;
        data_ready_callback();

        current_state = ST_IDLE;
    }

    wait_us(1000);
  }
}

// ----- SpiComms minimal accessors -----
linuxCncState_t volatile* SpiComms::get_linuxcnc_state() const { return this->linuxcnc_state; }
pruState_t volatile* SpiComms::get_pru_state() const { return this->pru_state; }
