#include "spi_comms.h"

#include <cstdio>
#include <cstring>

#include "LPC17xx.h"
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

constexpr uint32_t kPconpGpdma = 1UL << 29;
constexpr uint32_t kBurst4 = 1;       // burst size 4 (encoding for SBSize/DBSize)
constexpr uint32_t kWidthByte = 0;    // byte width (encoding for SWidth/DWidth)
constexpr uint32_t kSsp0TxConn = 0;   // SSP0 TX (Table 544)
constexpr uint32_t kSsp0RxConn = 1;   // SSP0 RX
constexpr uint32_t kTransferM2p = 1;  // memory to peripheral (Table 566)
constexpr uint32_t kTransferP2m = 2;  // peripheral to memory

}  // namespace

// DMACCxControl register (UM10360 Table 564)
// [11:0]   TransferSize
// [14:12]  SBSize (source burst): 0=1, 1=4, 2=8, 3=16, 4=32, 5=64, 6=128, 7=256
// [17:15]  DBSize (dest burst): same encoding
// [20:18]  SWidth (source width): 0=byte, 1=halfword, 2=word
// [23:21]  DWidth (dest width): same encoding
// [26]     SI (source increment)
// [27]     DI (dest increment)
// [31]     I (terminal count interrupt enable)
uint32_t SpiComms::CtrlWord(const bool src_inc, const bool dst_inc) const {
  return tx_buffer_.Size() | (kBurst4 << 12) | (kBurst4 << 15) | (kWidthByte << 18) | (kWidthByte << 21) |
         (src_inc ? 1UL << 26 : 0) | (dst_inc ? 1UL << 27 : 0) | (1UL << 31);
}

// DMACCxConfig register (UM10360 Table 565)
// [0]      E (channel enable)
// [5:1]    SrcPeripheral (Table 544)
// [10:6]   DestPeripheral (Table 544)
// [13:11]  TransferType (Table 566): 0=m2m, 1=m2p, 2=p2m, 3=p2p
// [14]     IE (error interrupt mask)
// [15]     ITC (terminal count interrupt mask)
uint32_t SpiComms::ConfigWord(const uint32_t transfer_type, const uint32_t conn) {
  return (1UL << 14) | (1UL << 15) | ((transfer_type & 0x7) << 11) | ((conn & 0x1F) << 1) | ((conn & 0x1F) << 6);
}

SpiComms::SpiComms() {
  if ((LPC_SC->PCONP & kPconpGpdma) && (LPC_GPDMA->DMACConfig & 1)) {
    error("DMA controller already initialized");
  }

  LPC_SC->PCONP |= kPconpGpdma;
  LPC_GPDMA->DMACConfig = 1;

  LPC_GPDMACH0->DMACCConfig = 0;
  LPC_GPDMACH1->DMACCConfig = 0;

  LPC_GPDMA->DMACIntTCClear = 0xFF;
  LPC_GPDMA->DMACIntErrClr = 0xFF;

  NVIC_SetVector(DMA_IRQn, reinterpret_cast<uint32_t>(DmaIrqHandler));
  NVIC_EnableIRQ(DMA_IRQn);

  tx_lli_.src = reinterpret_cast<uint32_t>(tx_buffer_.bytes);
  tx_lli_.dst = reinterpret_cast<uint32_t>(&LPC_SSP0->DR);
  tx_lli_.next = reinterpret_cast<uint32_t>(&tx_lli_);
  tx_lli_.ctrl = CtrlWord(true, false);

  rx_lli_.src = reinterpret_cast<uint32_t>(&LPC_SSP0->DR);
  rx_lli_.dst = reinterpret_cast<uint32_t>(rx_buffer_.bytes);
  rx_lli_.next = reinterpret_cast<uint32_t>(&rx_lli_);
  rx_lli_.ctrl = CtrlWord(false, true);

  tx_buffer_.AsStruct().packet_counter = kInitCounter;
}

void SpiComms::Start() {
  new SPISlave(kSpiMosi, kSpiMiso, kSpiSck, kSpiSsel);

  const auto tx = LPC_GPDMACH0;
  const auto rx = LPC_GPDMACH1;

  // TX channel (channel 0, memory to peripheral)
  LPC_GPDMA->DMACIntTCClear = 1;
  LPC_GPDMA->DMACIntErrClr = 1;
  tx->DMACCControl = 0;
  tx->DMACCConfig = 0;
  tx->DMACCLLI = reinterpret_cast<uint32_t>(&Instance()->tx_lli_);
  tx->DMACCSrcAddr = Instance()->tx_lli_.src;
  tx->DMACCDestAddr = Instance()->tx_lli_.dst;
  tx->DMACCControl = Instance()->tx_lli_.ctrl;
  tx->DMACCConfig = ConfigWord(kTransferM2p, kSsp0TxConn);

  // RX channel (channel 1, peripheral to memory)
  LPC_GPDMA->DMACIntTCClear = 2;
  LPC_GPDMA->DMACIntErrClr = 2;
  rx->DMACCControl = 0;
  rx->DMACCConfig = 0;
  rx->DMACCLLI = reinterpret_cast<uint32_t>(&Instance()->rx_lli_);
  rx->DMACCSrcAddr = Instance()->rx_lli_.src;
  rx->DMACCDestAddr = Instance()->rx_lli_.dst;
  rx->DMACCControl = Instance()->rx_lli_.ctrl;
  rx->DMACCConfig = ConfigWord(kTransferP2m, kSsp0RxConn);

  LPC_GPDMA->DMACConfig = 1;
  tx->DMACCConfig |= 1;
  rx->DMACCConfig |= 1;

  LPC_SSP0->DMACR = (1 << 1) | (1 << 0);
}

void SpiComms::DmaIrqHandler() {
  auto* self = Instance();

  if (LPC_GPDMA->DMACIntTCStat & 1) {
    self->TxCallback();
    LPC_GPDMA->DMACIntTCClear = 1;
  }
  if (LPC_GPDMA->DMACIntTCStat & 2) {
    self->RxCallback();
    LPC_GPDMA->DMACIntTCClear = 2;
  }
  if (LPC_GPDMA->DMACIntErrStat) {
    ErrCallback();
    LPC_GPDMA->DMACIntErrClr = 0x3;
  }
}

void SpiComms::TxCallback() { tx_buffer_.AsStruct().packet_counter++; }

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
