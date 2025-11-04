#include "spi_comms.h"

#define SPI_MOSI P0_18
#define SPI_MISO P0_17
#define SPI_SCK P0_15
#define SPI_SSEL P0_16

#define MAX_SPI_DELAY 5  // maximum number of (roughly) milliseconds without communication from LinuxCNC

enum State { ST_IDLE = 0, ST_RUNNING, ST_RESET };

SpiComms::SpiComms()
    : rx_dma1(new MODDMA_Config()), rx_dma2(new MODDMA_Config()), tx_dma(new MODDMA_Config()), tx_lli{} {
  // just initialize the peripheral, the communication is done through DMA
  new SPISlave(SPI_MOSI, SPI_MISO, SPI_SCK, SPI_SSEL);

  // Build TX LLI first (cyclic)
  this->tx_lli.srcAddr(reinterpret_cast<uint32_t>(tx_buffer.bytes))
      ->dstAddr(reinterpret_cast<uint32_t>(&LPC_SSP0->DR))
      ->control(dma.CxControl_TransferSize(tx_buffer.size()) | dma.CxControl_SBSize(MODDMA::_4) |
                dma.CxControl_DBSize(MODDMA::_4) | dma.CxControl_SWidth(MODDMA::byte) |
                dma.CxControl_DWidth(MODDMA::byte) | dma.CxControl_SI())
      ->nextLLI(reinterpret_cast<uint32_t>(&this->tx_lli));

  tx_dma->channelNum(MODDMA::Channel_0)
      ->srcMemAddr(reinterpret_cast<uint32_t>(tx_buffer.bytes))
      ->transferSize(tx_buffer.size())
      ->transferType(MODDMA::m2p)
      ->dstConn(MODDMA::SSP0_Tx)
      ->dmaLLI(reinterpret_cast<uint32_t>(&this->tx_lli))
      ->attach_err(this, &SpiComms::err_callback);

  rx_dma1->channelNum(MODDMA::Channel_2)
      ->dstMemAddr(reinterpret_cast<uint32_t>(rx_buffer1.bytes))
      ->transferSize(rx_buffer1.size())
      ->transferType(MODDMA::p2m)
      ->srcConn(MODDMA::SSP0_Rx)
      ->attach_tc(this, &SpiComms::rx1_callback)
      ->attach_err(this, &SpiComms::err_callback);

  rx_dma2->channelNum(MODDMA::Channel_3)
      ->dstMemAddr(reinterpret_cast<uint32_t>(rx_buffer2.bytes))
      ->transferSize(rx_buffer2.size())
      ->transferType(MODDMA::p2m)
      ->srcConn(MODDMA::SSP0_Rx)
      ->attach_tc(this, &SpiComms::rx2_callback)
      ->attach_err(this, &SpiComms::err_callback);

  // Initialize PRU->host packet counter
  this->tx_buffer.state().packet_counter = PRU_INIT_PKT;

  dma.Prepare(rx_dma1);
  dma.Prepare(tx_dma);

  // Pre-configure the alternate RX channel without enabling it
  dma.Setup(rx_dma2);

  // Enable SSP0 for DMA
  LPC_SSP0->DMACR = 0;
  LPC_SSP0->DMACR = 1 << 1 | 1 << 0;  // TX,RX DMA Enable
}

void SpiComms::rx1_callback() { this->rx_callback_impl(rx_buffer1.state(), this->rx_dma2); }

void SpiComms::rx2_callback() { this->rx_callback_impl(rx_buffer2.state(), this->rx_dma1); }

void SpiComms::data_ready_callback() {
  // trigger PendSV IRQ to signal that the data is ready
  SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

volatile LinuxCncState* SpiComms::get_linuxcnc_state() const { return this->linuxcnc_state; }
volatile PruState* SpiComms::get_pru_state() { return &this->tx_buffer.state(); }

// ReSharper disable once CppDFAUnreachableFunctionCall
void SpiComms::rx_callback_impl(const LinuxCncState& rx_buffer, MODDMA_Config* other_rx) {
  dma.Disable(dma.irqProcessingChannel());
  dma.clearTcIrq();

  // Check and move the received SPI data payload
  switch (rx_buffer.command) {
    case SpiCommand::Read:
      // Increment packet counter for the next reply
      this->tx_buffer.state().packet_counter++;
      data_ready = true;
      reject_count = 0;
      break;

    case SpiCommand::Write:
      data_ready = true;
      reject_count = 0;
      // don't copy the linuxcnc_state if the e-stop button is pressed
      if (!this->e_stop_active) {
        this->linuxcnc_state = const_cast<LinuxCncState*>(&rx_buffer);
        data_ready_callback();
      }
      break;

    default:
      if (reject_count++ > 5) {
        spi_error = true;
      }
  }

  // swap Rx buffers
  dma.Restart(other_rx);
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
        for (std::uint32_t i = 0; i < sizeof(LinuxCncState); ++i) {
          reinterpret_cast<volatile std::uint8_t*>(this->linuxcnc_state)[i] = 0;
        }
        data_ready_callback();

        current_state = ST_IDLE;
    }

    wait_us(1000);
  }
}
