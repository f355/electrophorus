#include "serial_comms.h"

#include "mbed.h"

namespace mbed {
FileHandle* mbed_override_console(int fd) {
  static UnbufferedSerial console(P2_8, P2_9, 115200);
  (void)fd;
  return &console;
}
}  // namespace mbed

#include "LPC17xx.h"
#include "machine_definitions.h"

static constexpr uint32_t UART_BAUD = 3000000;

static constexpr uint32_t UART_FCR_FIFO_EN = (1u << 0);
static constexpr uint32_t UART_FCR_RX_FIFO_RST = (1u << 1);
static constexpr uint32_t UART_FCR_TX_FIFO_RST = (1u << 2);
static constexpr uint32_t UART_FCR_DMA_MODE = (1u << 3);

SerialComms::SerialComms() {
  serial = new UnbufferedSerial(P0_2, P0_3, UART_BAUD);
  serial->format(8, SerialBase::None, 1);

  tx_buf[0].header = PRU_DATA;
  tx_buf[1].header = PRU_DATA;

  linuxcnc_state = &rx_buf[rx_ready_idx];
  pru_state = &tx_buf[tx_fill_idx];

  uart = (volatile LPC_UART_TypeDef*)LPC_UART0;

  // Enable FIFO and reset RX/TX (write-only FCR), and enable DMA mode
  uart->FCR = UART_FCR_FIFO_EN | UART_FCR_RX_FIFO_RST | UART_FCR_TX_FIFO_RST | UART_FCR_DMA_MODE;

  // Disable UART interrupts; DMA handles RX/TX
  uart->IER = 0;

  // Set explicit NVIC priority for DMA interrupt
  NVIC_SetPriority(DMA_IRQn, COMMS_DMA_PRIORITY);

  rx_phase = RxPhase::ExpectHeader;

  // Configure header RX channels (4 bytes, ping-pong between CH1 and CH2)
  rx_header_dma_cfg[0].channelNum(MODDMA::Channel_1)
      ->transferType(MODDMA::p2m)
      ->srcConn(MODDMA::UART0_Rx)
      ->dstMemAddr(reinterpret_cast<uint32_t>(&read_token_storage))
      ->transferSize(4)
      ->attach_tc(this, &SerialComms::on_rx_dma_tc);

  rx_header_dma_cfg[1].channelNum(MODDMA::Channel_2)
      ->transferType(MODDMA::p2m)
      ->srcConn(MODDMA::UART0_Rx)
      ->dstMemAddr(reinterpret_cast<uint32_t>(&read_token_storage))
      ->transferSize(4)
      ->attach_tc(this, &SerialComms::on_rx_dma_tc);

  // Configure payload RX channels (58 bytes, ping-pong between CH3 and CH4)
  rx_payload_dma_cfg[0].channelNum(MODDMA::Channel_3)
      ->transferType(MODDMA::p2m)
      ->srcConn(MODDMA::UART0_Rx)
      ->dstMemAddr(reinterpret_cast<uint32_t>(&rx_buf[0].buffer[4]))
      ->transferSize(XFER_BUF_SIZE - 4)
      ->attach_tc(this, &SerialComms::on_rx_dma_tc);

  rx_payload_dma_cfg[1].channelNum(MODDMA::Channel_4)
      ->transferType(MODDMA::p2m)
      ->srcConn(MODDMA::UART0_Rx)
      ->dstMemAddr(reinterpret_cast<uint32_t>(&rx_buf[1].buffer[4]))
      ->transferSize(XFER_BUF_SIZE - 4)
      ->attach_tc(this, &SerialComms::on_rx_dma_tc);

  // Configure TX DMA channel
  tx_dma_cfg.channelNum(MODDMA::Channel_0)
      ->transferType(MODDMA::m2p)
      ->srcMemAddr(reinterpret_cast<uint32_t>(&tx_buf[0].buffer[0]))
      ->dstConn(MODDMA::UART0_Tx)
      ->transferSize(XFER_BUF_SIZE)
      ->attach_tc(this, &SerialComms::on_tx_dma_tc);

  dma.Prepare(&rx_header_dma_cfg[0]);
  next_header_ch = 1;
}

void SerialComms::on_tx_dma_tc() {
  dma.Disable(dma.irqProcessingChannel());
  dma.clearTcIrq();
  tx_in_progress = false;
  tx_tc++;
}

void SerialComms::start_rx_dma_read_token() {
  header_rearm_calls++;
  // Directly write to channel registers to avoid modifying global GPDMA state
  uint32_t ch_num = rx_header_dma_cfg[next_header_ch].channelNum();
  LPC_GPDMACH_TypeDef* pChannel = (LPC_GPDMACH_TypeDef*)(LPC_GPDMACH0_BASE + (0x20 * ch_num));
  pChannel->DMACCDestAddr = reinterpret_cast<uint32_t>(&read_token_storage);
  pChannel->DMACCControl = (pChannel->DMACCControl & ~0xFFF) | 4;  // Update transfer size to 4
  pChannel->DMACCConfig |= 1;  // Enable
  header_prepare_calls++;
  next_header_ch ^= 1u;
}

void SerialComms::start_rx_dma_payload58() {
  rx_buf[rx_fill_idx].header = current_header;
  // Directly write to channel registers to avoid modifying global GPDMA state
  uint32_t ch_num = rx_payload_dma_cfg[rx_fill_idx].channelNum();
  LPC_GPDMACH_TypeDef* pChannel = (LPC_GPDMACH_TypeDef*)(LPC_GPDMACH0_BASE + (0x20 * ch_num));
  pChannel->DMACCDestAddr = reinterpret_cast<uint32_t>(&rx_buf[rx_fill_idx].buffer[4]);
  pChannel->DMACCControl = (pChannel->DMACCControl & ~0xFFF) | (XFER_BUF_SIZE - 4);  // Update transfer size to 58
  pChannel->DMACCConfig |= 1;  // Enable
  payload_prepare_calls++;
}

bool SerialComms::take_stepgen_conf(int axis, float* pos_scale, float* maxaccel, float* init_pos_mu) {
  const uint32_t mask = (1u << axis);
  if (!(conf_pending_mask & mask)) return false;
  core_util_critical_section_enter();
  *pos_scale = conf_position_scale[axis];
  *maxaccel = conf_max_accel[axis];
  *init_pos_mu = conf_init_pos_mu[axis];
  conf_pending_mask &= ~mask;
  core_util_critical_section_exit();
  return true;
}

void SerialComms::on_rx_dma_tc() {
  if (isr_active) {
    isr_reentry++;
    return;
  }
  isr_active = 1;

  dma.Disable(dma.irqProcessingChannel());
  dma.clearTcIrq();

  if (rx_phase == RxPhase::ExpectHeader) {
    current_header = read_token_storage;
    last_header = current_header;
    rx_header_tc++;
    switch (current_header) {
      case PRU_READ:
        read_token_ok_count++;
        tx_frames++;
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
          tx_dma_cfg.srcMemAddr(reinterpret_cast<uint32_t>(&tx_buf[tx_send_idx].buffer[0]));
          dma.Prepare(&tx_dma_cfg);
        } else {
          tx_skipped_busy++;
        }
        rx_phase = RxPhase::ExpectHeader;
        start_rx_dma_read_token();
        isr_active = 0;
        return;

      case PRU_DATA:
      case PRU_WRITE:
      case PRU_CONF:
        rx_phase = RxPhase::ExpectPayload;
        start_rx_dma_payload58();
        isr_active = 0;
        return;

      default:
        bad_header_count++;
        rx_phase = RxPhase::ExpectHeader;
        start_rx_dma_read_token();
        isr_active = 0;
        return;
    }
  }

  rx_payload_tc++;
  const uint32_t hdr = current_header;
  if (hdr == PRU_DATA) {
    const uint8_t ready_idx = rx_fill_idx;
    core_util_critical_section_enter();
    __DMB();
    linuxcnc_state = &rx_buf[ready_idx];
    __DMB();
    core_util_critical_section_exit();
    rx_ready_idx = ready_idx;
    rx_fill_idx = ready_idx ^ 1u;
    rx_frames++;
    this->data_ready_callback();

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
      tx_dma_cfg.srcMemAddr(reinterpret_cast<uint32_t>(&tx_buf[tx_send_idx].buffer[0]));
      dma.Prepare(&tx_dma_cfg);
    }
  } else if (hdr == PRU_WRITE) {
    const uint8_t ready_idx = rx_fill_idx;
    core_util_critical_section_enter();
    __DMB();
    linuxcnc_state = &rx_buf[ready_idx];
    __DMB();
    core_util_critical_section_exit();
    rx_ready_idx = ready_idx;
    rx_fill_idx = ready_idx ^ 1u;
    rx_frames++;
    this->data_ready_callback();
  } else if (hdr == PRU_CONF) {
    const volatile linuxCncConf_t* c = reinterpret_cast<volatile linuxCncConf_t*>(&rx_buf[rx_fill_idx]);
    for (int i = 0; i < STEPGENS; ++i) {
      conf_position_scale[i] = c->stepper_position_scale[i];
      conf_max_accel[i] = c->stepper_max_accel[i];
      conf_init_pos_mu[i] = c->stepper_init_position[i];
    }
    servo_period_s = c->servo_period_s;
    conf_pending_mask = (STEPGENS >= 32) ? 0xFFFFFFFFu : ((1u << STEPGENS) - 1u);
    rx_fill_idx ^= 1u;
    rx_frames++;
    this->data_ready_callback();
  } else {
    bad_header_count++;
    rx_fill_idx ^= 1u;
  }

  rx_phase = RxPhase::ExpectHeader;
  start_rx_dma_read_token();
  isr_active = 0;
}
