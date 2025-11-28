#include "dma.h"

#include "LPC17xx.h"

namespace {

constexpr uint32_t kPconpGpdma = 1UL << 29;
constexpr uint32_t kBurst4 = 1;
constexpr uint32_t kWidthByte = 0;
constexpr uint32_t kSsp0TxConn = 0;
constexpr uint32_t kSsp0RxConn = 1;
constexpr uint32_t kTransferM2p = 1;
constexpr uint32_t kTransferP2m = 2;

constexpr uint32_t CtrlWord(const size_t size, const bool src_inc, const bool dst_inc) {
  return (size & 0xFFF) | kBurst4 << 12 | kBurst4 << 15 | kWidthByte << 18 | kWidthByte << 21 |
         (src_inc ? 1UL << 26 : 0) | (dst_inc ? 1UL << 27 : 0) | 1UL << 31;
}

constexpr uint32_t ConfigWord(const uint32_t transfer_type, const uint32_t conn) {
  return 1UL << 14 | 1UL << 15 | (transfer_type & 0x7) << 11 | (conn & 0x1F) << 1 | (conn & 0x1F) << 6;
}

}  // namespace

SspDma* SspDma::Instance() {
  static SspDma instance;
  return &instance;
}

SspDma::SspDma() {
  if ((LPC_SC->PCONP & kPconpGpdma) && (LPC_GPDMA->DMACConfig & 1)) {
    error("DMA controller already initialized");
  }

  LPC_SC->PCONP |= kPconpGpdma;
  LPC_GPDMA->DMACConfig = 1;

  LPC_GPDMACH0->DMACCConfig = 0;
  LPC_GPDMACH1->DMACCConfig = 0;

  LPC_GPDMA->DMACIntTCClear = 0xFF;
  LPC_GPDMA->DMACIntErrClr = 0xFF;

  NVIC_SetVector(DMA_IRQn, reinterpret_cast<uint32_t>(IrqHandler));
  NVIC_EnableIRQ(DMA_IRQn);
}

void SspDma::ConfigureTx(const void* buffer, size_t size, const Callback<void()>& on_complete) {
  tx_cb_ = on_complete;
  tx_lli_.src = reinterpret_cast<uint32_t>(buffer);
  tx_lli_.dst = reinterpret_cast<uint32_t>(&LPC_SSP0->DR);
  tx_lli_.next = reinterpret_cast<uint32_t>(&tx_lli_);
  tx_lli_.ctrl = CtrlWord(size, true, false);
}

void SspDma::ConfigureRx(void* buffer, const size_t size, const Callback<void()>& on_complete) {
  rx_cb_ = on_complete;
  rx_lli_.src = reinterpret_cast<uint32_t>(&LPC_SSP0->DR);
  rx_lli_.dst = reinterpret_cast<uint32_t>(buffer);
  rx_lli_.next = reinterpret_cast<uint32_t>(&rx_lli_);
  rx_lli_.ctrl = CtrlWord(size, false, true);
}

void SspDma::SetErrorCallback(const Callback<void()>& on_error) { err_cb_ = on_error; }

void SspDma::Start() {
  LPC_GPDMACH_TypeDef* tx = LPC_GPDMACH0;
  LPC_GPDMACH_TypeDef* rx = LPC_GPDMACH1;

  // TX channel (channel 0, memory to peripheral)
  LPC_GPDMA->DMACIntTCClear = 1;
  LPC_GPDMA->DMACIntErrClr = 1;
  tx->DMACCControl = 0;
  tx->DMACCConfig = 0;
  tx->DMACCLLI = reinterpret_cast<uint32_t>(&tx_lli_);
  tx->DMACCSrcAddr = tx_lli_.src;
  tx->DMACCDestAddr = tx_lli_.dst;
  tx->DMACCControl = tx_lli_.ctrl;
  tx->DMACCConfig = ConfigWord(kTransferM2p, kSsp0TxConn);

  // RX channel (channel 1, peripheral to memory)
  LPC_GPDMA->DMACIntTCClear = 2;
  LPC_GPDMA->DMACIntErrClr = 2;
  rx->DMACCControl = 0;
  rx->DMACCConfig = 0;
  rx->DMACCLLI = reinterpret_cast<uint32_t>(&rx_lli_);
  rx->DMACCSrcAddr = rx_lli_.src;
  rx->DMACCDestAddr = rx_lli_.dst;
  rx->DMACCControl = rx_lli_.ctrl;
  rx->DMACCConfig = ConfigWord(kTransferP2m, kSsp0RxConn);

  LPC_GPDMA->DMACConfig = 1;
  while (!(LPC_GPDMA->DMACConfig & 1)) {
  }

  tx->DMACCConfig |= 1;
  rx->DMACCConfig |= 1;

  LPC_SSP0->DMACR = (1 << 1) | (1 << 0);
}

void SspDma::IrqHandler() {
  const auto* self = Instance();

  // Channel 0: TX
  if (LPC_GPDMA->DMACIntTCStat & 1) {
    if (self->tx_cb_) self->tx_cb_();
    LPC_GPDMA->DMACIntTCClear = 1;
  }
  if (LPC_GPDMA->DMACIntErrStat & 1) {
    if (self->err_cb_) self->err_cb_();
    LPC_GPDMA->DMACIntErrClr = 1;
  }

  // Channel 1: RX
  if (LPC_GPDMA->DMACIntTCStat & 2) {
    if (self->rx_cb_) self->rx_cb_();
    LPC_GPDMA->DMACIntTCClear = 2;
  }
  if (LPC_GPDMA->DMACIntErrStat & 2) {
    if (self->err_cb_) self->err_cb_();
    LPC_GPDMA->DMACIntErrClr = 2;
  }
}
