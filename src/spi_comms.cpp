#include "spi_comms.h"

#define SPI_MOSI P0_18
#define SPI_MISO P0_17
#define SPI_SCK P0_15
#define SPI_SSEL P0_16

#define MAX_SPI_DELAY 5  // maximum number of (roughly) milliseconds without communication from LinuxCNC

enum State { ST_IDLE = 0, ST_RUNNING, ST_RESET };

SpiComms::SpiComms()
    : rx_dma1(new MODDMA_Config()),
      rx_dma2(new MODDMA_Config()),
      tx_dma1(new MODDMA_Config()),
      tx_dma2(new MODDMA_Config()),
      linuxcnc_state(new linuxCncState_t()),
      pru_state(new pruState_t()) {
  // just initialize the peripheral, the communication is done through DMA
  new SPISlave(SPI_MOSI, SPI_MISO, SPI_SCK, SPI_SSEL);

  tx_dma1->channelNum(MODDMA::Channel_0)
      ->srcMemAddr(reinterpret_cast<uint32_t>(pru_state))
      ->dstMemAddr(0)
      ->transferSize(SPI_BUF_SIZE)
      ->transferType(MODDMA::m2p)
      ->srcConn(0)
      ->dstConn(MODDMA::SSP0_Tx)
      ->attach_tc(this, &SpiComms::tx1_callback)
      ->attach_err(this, &SpiComms::err_callback);

  tx_dma2->channelNum(MODDMA::Channel_1)
      ->srcMemAddr(reinterpret_cast<uint32_t>(pru_state))
      ->dstMemAddr(0)
      ->transferSize(SPI_BUF_SIZE)
      ->transferType(MODDMA::m2p)
      ->srcConn(0)
      ->dstConn(MODDMA::SSP0_Tx)
      ->attach_tc(this, &SpiComms::tx2_callback)
      ->attach_err(this, &SpiComms::err_callback);

  rx_dma1->channelNum(MODDMA::Channel_2)
      ->srcMemAddr(0)
      ->dstMemAddr(reinterpret_cast<uint32_t>(&temp_rx_buffer1))
      ->transferSize(SPI_BUF_SIZE)
      ->transferType(MODDMA::p2m)
      ->srcConn(MODDMA::SSP0_Rx)
      ->dstConn(0)
      ->attach_tc(this, &SpiComms::rx1_callback)
      ->attach_err(this, &SpiComms::err_callback);

  rx_dma2->channelNum(MODDMA::Channel_3)
      ->srcMemAddr(0)
      ->dstMemAddr(reinterpret_cast<uint32_t>(&temp_rx_buffer2))
      ->transferSize(SPI_BUF_SIZE)
      ->transferType(MODDMA::p2m)
      ->srcConn(MODDMA::SSP0_Rx)
      ->dstConn(0)
      ->attach_tc(this, &SpiComms::rx2_callback)
      ->attach_err(this, &SpiComms::err_callback);

  NVIC_SetPriority(DMA_IRQn, DMA_PRIORITY);

  this->pru_state->header = PRU_DATA;

  // Pass the configurations to the controller
  dma.Prepare(rx_dma1);
  dma.Prepare(tx_dma1);

  // Enable SSP0 for DMA
  LPC_SSP0->DMACR = 0;
  LPC_SSP0->DMACR = 1 << 1 | 1 << 0;  // TX,RX DMA Enable
}

void SpiComms::tx1_callback() {
  dma.Disable(dma.irqProcessingChannel());
  dma.clearTcIrq();
  dma.Prepare(tx_dma2);
}

void SpiComms::tx2_callback() {
  dma.Disable(dma.irqProcessingChannel());
  dma.clearTcIrq();
  dma.Prepare(tx_dma1);
}

void SpiComms::rx1_callback() { this->rx_callback_impl(this->temp_rx_buffer1, this->rx_dma2); }

void SpiComms::rx2_callback() { this->rx_callback_impl(this->temp_rx_buffer2, this->rx_dma1); }

void SpiComms::data_ready_callback() {
  // trigger PendSV IRQ to signal that the data is ready
  SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

volatile linuxCncState_t* SpiComms::get_linuxcnc_state() const { return this->linuxcnc_state; }
volatile pruState_t* SpiComms::get_pru_state() const { return this->pru_state; }

// ReSharper disable once CppDFAUnreachableFunctionCall
void SpiComms::rx_callback_impl(const linuxCncState_t& rx_buffer, MODDMA_Config* other_rx) {
  dma.Disable(dma.irqProcessingChannel());
  dma.clearTcIrq();

  data_ready = false;
  spi_error = false;

  // Check and move the received SPI data payload
  switch (rx_buffer.header) {
    case PRU_READ:
      data_ready = true;
      reject_count = 0;
      break;

    case PRU_WRITE:
      data_ready = true;
      reject_count = 0;
      // don't copy the linuxcnc_state if the e-stop button is pressed
      if (!this->e_stop_active) {
        for (size_t i = 0; i < SPI_BUF_SIZE; i++) {
          this->linuxcnc_state->buffer[i] = rx_buffer.buffer[i];
        }
        data_ready_callback();
      }
      break;

    default:
      reject_count++;
      if (reject_count > 5) {
        spi_error = true;
      }
  }

  // swap Rx buffers
  dma.Prepare(other_rx);
}

void SpiComms::err_callback() { error("DMA error on channel %d!\n", dma.irqProcessingChannel()); }

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

        // set the whole rxData buffer to 0
        // can't memset volatile memory, so use a loop instead
        for (volatile uint8_t& b : this->linuxcnc_state->buffer) b = 0;
        data_ready_callback();

        current_state = ST_IDLE;
    }

    wait_us(1000);
  }
}

static_assert(sizeof(linuxCncState_t) == sizeof(pruState_t), "rx and tx buffer size mismatch!");
static_assert(sizeof(linuxCncState_t) <= SPI_BUF_SIZE, "rx buffer too small!");
static_assert(sizeof(pruState_t) <= SPI_BUF_SIZE, "tx buffer too small!");
