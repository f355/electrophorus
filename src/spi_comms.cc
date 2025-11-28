#include "spi_comms.h"

#include "dma/dma.h"
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
  auto* dma = SspDma::Instance();
  dma->ConfigureTx(tx_buffer_.bytes, tx_buffer_.Size(), callback(this, &SpiComms::TxCallback));
  dma->ConfigureRx(rx_buffer_.bytes, rx_buffer_.Size(), callback(this, &SpiComms::RxCallback));
  dma->SetErrorCallback(callback(&SpiComms::ErrCallback));

  tx_buffer_.AsStruct().packet_counter = kInitCounter;
}

void SpiComms::Start() {
  new SPISlave(kSpiMosi, kSpiMiso, kSpiSck, kSpiSsel);
  SspDma::Instance()->Start();
}

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

void SpiComms::EStop(const bool active) {
  e_stop_active_ = active;
  if (active) RxListener::HandleRxDeferred();
}

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
      if (e_stop_active_)
        printf("e-stop pressed, machine halted!\n");
      else
        printf("e-stop released, resuming operation...\n");

      prev_e_stop_active = e_stop_active_;
    }

    if (spi_error_) {
      printf("SPI communication error, ignoring. If you see a lot of these, check your cabling.\n");
      spi_error_ = false;
    }

    switch (current_state) {
      case StIdle:
        if (current_state != prev_state) printf("waiting for LinuxCNC...\n");
        prev_state = current_state;

        if (data_ready_) current_state = StRunning;

        break;

      case StRunning:
        if (current_state != prev_state) printf("running...\n");
        prev_state = current_state;

        if (data_ready_) {
          spi_delay = 0;
          data_ready_ = false;
        } else
          spi_delay++;

        if (spi_delay > kMaxSpiDelay) {
          printf("no communication from LinuxCNC, e-stop active?\n");
          spi_delay = 0;
          current_state = StReset;
        }
        break;
      case StReset:
        if (current_state != prev_state) printf("resetting receive buffer...\n");
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
