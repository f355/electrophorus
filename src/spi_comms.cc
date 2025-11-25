#include "spi_comms.h"

#include "MODDMA.h"
#include "mbed.h"
#include "rx_listener.h"
#include "spi_protocol/spi_protocol.h"

namespace {
constexpr auto kSpiMosi = P0_18;
constexpr auto kSpiMiso = P0_17;
constexpr auto kSpiSck = P0_15;
constexpr auto kSpiSsel = P0_16;

constexpr auto kMaxSpiDelay = 5;  // maximum number of (roughly) milliseconds without communication from LinuxCNC

enum State { StIdle, StRunning, StReset };
}  // namespace

SpiComms::SpiComms() {
  rx_lli_.dstAddr(reinterpret_cast<uint32_t>(rx_buffer_.bytes))
      ->srcAddr(reinterpret_cast<uint32_t>(&LPC_SSP0->DR))
      ->control(dma_.CxControl_SBSize(MODDMA::_4) | dma_.CxControl_SWidth(MODDMA::byte) |
                dma_.CxControl_DBSize(MODDMA::_4) | dma_.CxControl_DWidth(MODDMA::byte) |
                dma_.CxControl_TransferSize(rx_buffer_.Size()) | dma_.CxControl_DI() | dma_.CxControl_I())
      ->nextLLI(reinterpret_cast<uint32_t>(&rx_lli_));

  tx_lli_.srcAddr(reinterpret_cast<uint32_t>(tx_buffer_.bytes))
      ->dstAddr(reinterpret_cast<uint32_t>(&LPC_SSP0->DR))
      ->control(dma_.CxControl_SBSize(MODDMA::_4) | dma_.CxControl_SWidth(MODDMA::byte) |
                dma_.CxControl_DBSize(MODDMA::_4) | dma_.CxControl_DWidth(MODDMA::byte) |
                dma_.CxControl_TransferSize(tx_buffer_.Size()) | dma_.CxControl_SI() | dma_.CxControl_I())
      ->nextLLI(reinterpret_cast<uint32_t>(&tx_lli_));

  tx_dma_.channelNum(MODDMA::Channel_0)
      ->srcMemAddr(reinterpret_cast<uint32_t>(tx_buffer_.bytes))
      ->transferSize(tx_buffer_.Size())
      ->transferType(MODDMA::m2p)
      ->dstConn(MODDMA::SSP0_Tx)
      ->dmaLLI(reinterpret_cast<uint32_t>(&tx_lli_))
      ->attach_tc(this, &SpiComms::TxCallback)
      ->attach_err(this, &SpiComms::ErrCallback);

  rx_dma_.channelNum(MODDMA::Channel_1)
      ->dstMemAddr(reinterpret_cast<uint32_t>(rx_buffer_.bytes))
      ->transferSize(rx_buffer_.Size())
      ->transferType(MODDMA::p2m)
      ->srcConn(MODDMA::SSP0_Rx)
      ->dmaLLI(reinterpret_cast<uint32_t>(&rx_lli_))
      ->attach_tc(this, &SpiComms::RxCallback)
      ->attach_err(this, &SpiComms::ErrCallback);

  // Initialize PRU->host packet counter
  tx_buffer_.AsStruct().packet_counter = kInitCounter;
}

void SpiComms::Start() {
  // just initialize the peripheral, the communication is done through DMA
  new SPISlave(kSpiMosi, kSpiMiso, kSpiSck, kSpiSsel);

  dma_.Prepare(&rx_dma_);
  dma_.Prepare(&tx_dma_);

  // Enable SSP0 for DMA
  LPC_SSP0->DMACR = 0;
  LPC_SSP0->DMACR = 1 << 1 | 1 << 0;  // TX,RX DMA Enable
}

// ReSharper disable once CppDFAUnreachableFunctionCall
void SpiComms::RxCallback() {
  switch (rx_buffer_.AsStruct().command) {
    case SpiCommand::Write: {
      // expose the received data only if the e-stop button isn't pressed
      if (!e_stop_active_) memcpy(linuxcnc_state_.bytes, rx_buffer_.bytes, sizeof(LinuxCncState));
      RxListener::HandleRxDeferred();
      [[fallthrough]];
    }
    case SpiCommand::Read: {
      data_ready_ = true;
      reject_count_ = 0;
      break;
    }
    default:
      if (reject_count_++ > 5) spi_error_ = true;
  }
}

void SpiComms::TxCallback() { tx_buffer_.AsStruct().packet_counter++; }

SpiComms* SpiComms::Instance() {
  static SpiComms instance;
  return &instance;
}

void SpiComms::Loop() {
  uint8_t spi_delay = 0;
  State current_state = StIdle;
  State prev_state = StReset;
  bool prev_e_stop_active = e_stop_active_;

  while (true) {
    Watchdog::get_instance().kick();

    if (e_stop_active_ != prev_e_stop_active) {
      if (e_stop_active_) {
        printf("e-stop pressed, machine halted!\n");
      } else {
        printf("e-stop released, resuming operation...\n");
      }
      prev_e_stop_active = e_stop_active_;
    }

    if (spi_error_) {
      printf("SPI communication error, ignoring. If you see a lot of these, check your cabling.\n");
      spi_error_ = false;
    }

    switch (current_state) {
      case StIdle:
        if (current_state != prev_state) {
          printf("waiting for LinuxCNC...\n");
        }
        prev_state = current_state;

        if (data_ready_) {
          current_state = StRunning;
        }
        break;

      case StRunning:
        if (current_state != prev_state) {
          printf("running...\n");
        }
        prev_state = current_state;

        if (data_ready_) {
          spi_delay = 0;
          data_ready_ = false;
        } else {
          spi_delay++;
        }

        if (spi_delay > kMaxSpiDelay) {
          printf("no communication from LinuxCNC, e-stop active?\n");
          spi_delay = 0;
          current_state = StReset;
        }
        break;
      case StReset:
        if (current_state != prev_state) {
          printf("resetting receive buffer...\n");
        }
        prev_state = current_state;

        // set the whole rxData buffer to 0
        // can't memset volatile memory, so use a loop instead
        for (size_t i = 0; i < sizeof(LinuxCncState); ++i) linuxcnc_state_.bytes[i] = 0;

        RxListener::HandleRxDeferred();

        current_state = StIdle;
    }

    wait_us(1000);
  }
}
