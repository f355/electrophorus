#include "serial_comms.h"

#include <cstring>

#include "LPC17xx.h"
#include "machine_definitions.h"

#ifndef EPHO_COMMS_UART
#define EPHO_COMMS_UART 2
#endif

#if EPHO_COMMS_UART == 0
static constexpr PinName COMMS_TX_PIN = P0_2;
static constexpr PinName COMMS_RX_PIN = P0_3;
static constexpr MODDMA::GPDMA_CONNECTION COMMS_DMA_TX = MODDMA::UART0_Tx;
#elif EPHO_COMMS_UART == 2
static constexpr PinName COMMS_TX_PIN = P2_8;
static constexpr PinName COMMS_RX_PIN = P2_9;
static constexpr MODDMA::GPDMA_CONNECTION COMMS_DMA_TX = MODDMA::UART2_Tx;
#else
#error "EPHO_COMMS_UART must be 0 or 2"
#endif

static constexpr uint32_t UART_BAUD = 3000000;

static constexpr uint32_t UART_FCR_FIFO_EN = (1u << 0);
static constexpr uint32_t UART_FCR_RX_FIFO_RST = (1u << 1);
static constexpr uint32_t UART_FCR_TX_FIFO_RST = (1u << 2);
static constexpr uint32_t UART_FCR_DMA_MODE = (1u << 3);

static uint32_t crc32_ieee(const volatile uint8_t* data, const size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    const uint32_t v = data[i];
    crc ^= static_cast<uint8_t>(v);
    for (int b = 0; b < 8; ++b) {
      const uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

SerialComms::SerialComms() : tx_dma1(new MODDMA_Config()), tx_dma2(new MODDMA_Config()) {
  serial = new BufferedSerial(COMMS_TX_PIN, COMMS_RX_PIN, UART_BAUD);
  serial->set_blocking(false);

// Enable FIFO and reset RX/TX; keep DMA mode for TX; do not assert THRE
#if EPHO_COMMS_UART == 0
  LPC_UART0->FCR = UART_FCR_FIFO_EN | UART_FCR_RX_FIFO_RST | UART_FCR_TX_FIFO_RST | UART_FCR_DMA_MODE;
  LPC_UART0->IER &= ~(1u << 1); // clear THRE; preserve other bits
#elif EPHO_COMMS_UART == 2
  LPC_UART2->FCR = UART_FCR_FIFO_EN | UART_FCR_RX_FIFO_RST | UART_FCR_TX_FIFO_RST | UART_FCR_DMA_MODE;
  LPC_UART2->IER &= ~(1u << 1); // clear THRE; preserve other bits
#endif

  tx_buf1.header = PRU_DATA;
  tx_buf2.header = PRU_DATA;

  const volatile uint8_t* s = reinterpret_cast<const volatile uint8_t*>(&tx_buf1) + 4;
  tx_buf1.crc = crc32_ieee(s, sizeof(pruState_t) - 8);

  tx_dma1->channelNum(MODDMA::Channel_2)
      ->srcMemAddr(reinterpret_cast<uint32_t>(&tx_buf1))
      ->dstMemAddr(0)
      ->transferSize(sizeof(pruState_t))
      ->transferType(MODDMA::m2p)
      ->srcConn(0)
      ->dstConn(COMMS_DMA_TX)
      ->attach_tc(this, &SerialComms::on_tx_dma_tc1);

  tx_dma2->channelNum(MODDMA::Channel_3)
      ->srcMemAddr(reinterpret_cast<uint32_t>(&tx_buf2))
      ->dstMemAddr(0)
      ->transferSize(sizeof(pruState_t))
      ->transferType(MODDMA::m2p)
      ->srcConn(0)
      ->dstConn(COMMS_DMA_TX)
      ->attach_tc(this, &SerialComms::on_tx_dma_tc2);

  NVIC_SetPriority(DMA_IRQn, COMMS_DMA_PRIORITY);

  // RX moved to main-loop polling; do not start RX DMA here
  dma.Prepare(tx_dma1);
}

void SerialComms::on_tx_dma_tc1() { this->on_tx_dma(&tx_buf1, &tx_buf2, tx_dma2); }
void SerialComms::on_tx_dma_tc2() { this->on_tx_dma(&tx_buf2, &tx_buf1, tx_dma1); }

void SerialComms::on_tx_dma(volatile pruState_t* our_buf, volatile pruState_t* other_buf, MODDMA_Config* other_dma) {
  dma.Disable(dma.irqProcessingChannel());
  dma.clearTcIrq();

  ++tx_frames;

  pru_state = our_buf;

  const volatile uint8_t* start = reinterpret_cast<const volatile uint8_t*>(other_buf) + 4;
  other_buf->crc = crc32_ieee(start, sizeof(pruState_t) - 8);

  dma.Prepare(other_dma);
}

void SerialComms::clear_linuxcnc_state() const {
  volatile auto* p = reinterpret_cast<volatile uint8_t*>(linuxcnc_state);
  for (size_t i = 0; i < sizeof(linuxCncState_t); i++) p[i] = 0u;
}

void SerialComms::poll_rx_nonblocking() {
  // Check and count UART RX overrun (LSR.OE) before draining
#if EPHO_COMMS_UART == 0
  const uint32_t lsr = LPC_UART0->LSR;
#elif EPHO_COMMS_UART == 2
  const uint32_t lsr = LPC_UART2->LSR;
#endif
  if (lsr & (1u << 1)) {
    ++rx_overrun_errors;
    static uint32_t last_print = 0;
    if (rx_overrun_errors != last_print) {
      printf("UART RX overrun detected (LSR=0x%02x), total=%lu\n", (unsigned)lsr, (unsigned long)rx_overrun_errors);
      last_print = rx_overrun_errors;
    }
  }

  // Drain BufferedSerial ring buffer without relying on readable()
  while (rx_buf_end < RX_BUF_LEN) {
    ssize_t n = serial->read(rx_buf + rx_buf_end, RX_BUF_LEN - rx_buf_end);
    if (n > 0) {
      rx_buf_end += (size_t)n;
      continue;
    }
    break; // no more data (EAGAIN) or error
  }
  if (rx_buf_end < 4) return;

  // Forward scan for header; allow partial-frame tail to remain
  size_t i = 0;
  while (i + 4 <= rx_buf_end) {
    // Seek header
    if (*reinterpret_cast<const uint32_t*>(rx_buf + i) != PRU_WRITE) {
      ++i;
      continue;
    }
    // Have header at i; wait until we have full frame
    if (rx_buf_end - i < RX_FRAME_SIZE) break;

    // Validate frame at i
    const auto* frame = reinterpret_cast<const linuxCncState_t*>(rx_buf + i);
    const auto start = reinterpret_cast<const uint8_t*>(frame) + 4;
    if (frame->crc == crc32_ieee(start, RX_FRAME_SIZE - 8)) {
      // Copy out a valid frame and publish
      memcpy(rx_cpu_fill, rx_buf + i, RX_FRAME_SIZE);
      linuxcnc_state = rx_cpu_fill;
      ++rx_frames;
      data_ready_callback();
      rx_cpu_fill = (rx_cpu_fill == &rx_buf1) ? &rx_buf2 : &rx_buf1;
      // Consume bytes up to end of this frame
      i += RX_FRAME_SIZE;
      // Compact remaining bytes (if any)
      const size_t leftover_bytes = rx_buf_end - i;
      if (leftover_bytes) memmove(rx_buf, rx_buf + i, leftover_bytes);
      rx_buf_end = leftover_bytes;
      return;
    }

    // Bad frame: skip this header and continue searching
    ++rx_crc_fail;
    static int dbg = 0;
    if (dbg++ < 5) {
      const uint8_t* h = rx_buf + i;
      const uint32_t calc = crc32_ieee(h + 4, RX_FRAME_SIZE - 8);
      printf("crc_fail i=%u hdr=%02x %02x %02x %02x rx=%08x calc=%08x size=%u\n",
             (unsigned)i, h[0], h[1], h[2], h[3], frame->crc, calc, (unsigned)RX_FRAME_SIZE);
      printf("pkt:");
      for (size_t k = 0; k < RX_FRAME_SIZE; ++k) {
        printf(" %02x", h[k]);
      }
      printf("\n");
    }
    ++i;
  }

  // Drop bytes before the last 3 to allow header overlap
  if (i > 0) {
    const size_t keep = rx_buf_end - i;
    if (keep && i < rx_buf_end) memmove(rx_buf, rx_buf + i, keep);
    rx_buf_end = keep;
  }
}
