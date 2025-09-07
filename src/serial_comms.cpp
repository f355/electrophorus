#include "serial_comms.h"

#include "LPC17xx.h"
#include "machine_definitions.h"

static constexpr uint32_t UART_BAUD = 3000000;

static constexpr uint32_t UART_FCR_FIFO_EN = (1u << 0);
static constexpr uint32_t UART_FCR_RX_FIFO_RST = (1u << 1);
static constexpr uint32_t UART_FCR_TX_FIFO_RST = (1u << 2);

SerialComms::SerialComms() {
  serial = new UnbufferedSerial(P2_8, P2_9, UART_BAUD);
  serial->format(8, SerialBase::None, 1);

  tx_buf[0].header = PRU_DATA;
  tx_buf[1].header = PRU_DATA;

  linuxcnc_state = &rx_buf[rx_ready_idx];
  pru_state = &tx_buf[tx_fill_idx];

  uart = LPC_UART2;

  // Enable FIFO and reset RX/TX (write-only FCR)
  uart->FCR = UART_FCR_FIFO_EN | UART_FCR_RX_FIFO_RST | UART_FCR_TX_FIFO_RST;

  // Disable UART interrupts; DMA handles RX/TX
  uart->IER = 0;

  // Set explicit NVIC priority for DMA interrupt
  NVIC_SetPriority(DMA_IRQn, COMMS_DMA_PRIORITY);

  start_rx_dma_for_fill();
}

void SerialComms::on_tx_dma_tc() {
  // Clear TC IRQ and mark TX complete
  dma.clearTcIrq();
  tx_in_progress = false;
  // Arm next RX DMA now that TX has completed
  start_rx_dma_for_fill();
}

void SerialComms::start_rx_dma_for_fill() {
  rx_dma_cfg.channelNum(MODDMA::Channel_1)
      ->transferType(MODDMA::p2m)
      ->srcConn(MODDMA::UART2_Rx)
      ->dstMemAddr(reinterpret_cast<uint32_t>(&rx_buf[rx_fill_idx].buffer[0]))
      ->transferSize(XFER_BUF_SIZE)
      ->attach_tc(this, &SerialComms::on_rx_dma_tc);
  dma.Prepare(&rx_dma_cfg);
}

void SerialComms::on_rx_dma_tc() {
  dma.clearTcIrq();

  bool valid = (rx_buf[rx_fill_idx].header == PRU_DATA);

  // Start TX of the previous reply via DMA (once per slot)
  if (!tx_in_progress) {
    const uint8_t send_idx = (pru_state == &tx_buf[0]) ? 0u : 1u;
    core_util_critical_section_enter();
    __DMB();
    pru_state = &tx_buf[send_idx ^ 1u];
    __DMB();
    core_util_critical_section_exit();
    tx_send_idx = send_idx;
    tx_fill_idx = send_idx ^ 1u;

    tx_in_progress = true;

    tx_dma_cfg.channelNum(MODDMA::Channel_0)
        ->transferType(MODDMA::m2p)
        ->srcMemAddr(reinterpret_cast<uint32_t>(&tx_buf[tx_send_idx].buffer[0]))
        ->dstConn(MODDMA::UART2_Tx)
        ->transferSize(XFER_BUF_SIZE)
        ->attach_tc(this, &SerialComms::on_tx_dma_tc);
    dma.Prepare(&tx_dma_cfg);
  }

  // Publish or just tick to keep cadence
  if (valid) {
    const uint8_t ready_idx = rx_fill_idx;
    core_util_critical_section_enter();
    __DMB();
    linuxcnc_state = &rx_buf[ready_idx];
    __DMB();
    core_util_critical_section_exit();
    rx_ready_idx = ready_idx;
    rx_fill_idx = ready_idx ^ 1u;
    this->data_ready_callback();
  } else {
    // keep cadence without publishing new state
    this->data_ready_callback();
    // rotate fill buffer anyway to avoid overwriting same slot repeatedly
    rx_fill_idx ^= 1u;
  }

  // RX re-armed from TX DMA TC to keep half-duplex cadence
}
