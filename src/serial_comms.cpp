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

#ifndef MBED_CONF_APP_SAME_TICK_MODE
#define MBED_CONF_APP_SAME_TICK_MODE 0
#endif

SerialComms::SerialComms() {
  serial = new UnbufferedSerial(P0_2, P0_3, UART_BAUD);
  serial->format(8, SerialBase::None, 1);

  tx_buf[0].header = PRU_DATA;
  tx_buf[1].header = PRU_DATA;

  linuxcnc_state = &rx_buf[rx_ready_idx];
  pru_state = &tx_buf[tx_fill_idx];

  uart = (volatile LPC_UART_TypeDef*)LPC_UART0;

  // Enable FIFO and reset RX/TX (write-only FCR)
  uart->FCR = UART_FCR_FIFO_EN | UART_FCR_RX_FIFO_RST | UART_FCR_TX_FIFO_RST;

  // Disable UART interrupts; DMA handles RX/TX
  uart->IER = 0;

  // Set explicit NVIC priority for DMA interrupt
  NVIC_SetPriority(DMA_IRQn, COMMS_DMA_PRIORITY);

  same_tick_mode = (MBED_CONF_APP_SAME_TICK_MODE != 0);
  if (same_tick_mode) {
    rx_phase = RxPhase::ExpectRead;
    start_rx_dma_read_token();
  } else {
    start_rx_dma_for_fill();
  }
}

void SerialComms::on_tx_dma_tc() {
  // Clear TC IRQ and mark TX complete
  dma.clearTcIrq();
  tx_in_progress = false;
  // In pipelined (non same-tick) mode, arm next RX now that TX has completed
  if (!same_tick_mode) {
    start_rx_dma_for_fill();
  }
}

void SerialComms::start_rx_dma_for_fill() {
  rx_dma_cfg.channelNum(MODDMA::Channel_1)
      ->transferType(MODDMA::p2m)
      ->srcConn(MODDMA::UART0_Rx)
      ->dstMemAddr(reinterpret_cast<uint32_t>(&rx_buf[rx_fill_idx].buffer[0]))
      ->transferSize(XFER_BUF_SIZE)
      ->attach_tc(this, &SerialComms::on_rx_dma_tc);
  dma.Prepare(&rx_dma_cfg);
}

void SerialComms::start_rx_dma_read_token() {
  rx_dma_cfg.channelNum(MODDMA::Channel_1)
      ->transferType(MODDMA::p2m)
      ->srcConn(MODDMA::UART0_Rx)
      ->dstMemAddr(reinterpret_cast<uint32_t>(&read_token_storage))
      ->transferSize(4)
      ->attach_tc(this, &SerialComms::on_rx_dma_tc);
  dma.Prepare(&rx_dma_cfg);
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
  dma.clearTcIrq();

  if (same_tick_mode) {
    if (rx_phase == RxPhase::ExpectRead) {
      // Host sent 4-byte PRU_READ token; reply immediately with current tx frame
      if (read_token_storage != PRU_READ) {
        bad_header_count++;
      } else {
        read_token_ok_count++;
      }
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
            ->dstConn(MODDMA::UART0_Tx)
            ->transferSize(XFER_BUF_SIZE)
            ->attach_tc(this, &SerialComms::on_tx_dma_tc);
        dma.Prepare(&tx_dma_cfg);
      }
      // Now expect a full 62-byte command/config frame
      rx_phase = RxPhase::ExpectCmd;
      start_rx_dma_for_fill();
      return;
    } else {
      // Received 62-byte command/config
      const bool valid = (rx_buf[rx_fill_idx].header == PRU_DATA);
      const bool is_conf = (rx_buf[rx_fill_idx].header == PRU_CONF);

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
        if (is_conf) {
          const volatile linuxCncConf_t* c = reinterpret_cast<volatile linuxCncConf_t*>(&rx_buf[rx_fill_idx]);
          for (int i = 0; i < STEPGENS; ++i) {
            conf_position_scale[i] = c->stepper_position_scale[i];
            conf_max_accel[i] = c->stepper_max_accel[i];
            conf_init_pos_mu[i] = c->stepper_init_position[i];
          }
          servo_period_s = c->servo_period_s;
          conf_pending_mask = (STEPGENS >= 32) ? 0xFFFFFFFFu : ((1u << STEPGENS) - 1u);
        } else {
          bad_header_count++;
        }
        this->data_ready_callback();
        rx_fill_idx ^= 1u;
      }
      // After command, expect next 4-byte read token
      rx_phase = RxPhase::ExpectRead;
      start_rx_dma_read_token();
      return;
    }
  }

  // Pipelined (previous) mode
  const bool valid = (rx_buf[rx_fill_idx].header == PRU_DATA);
  const bool is_conf = (rx_buf[rx_fill_idx].header == PRU_CONF);

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
        ->dstConn(MODDMA::UART0_Tx)
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
    // Config frame? Capture values and expose for modules to consume on next on_rx.
    if (is_conf) {
      const volatile linuxCncConf_t* c = reinterpret_cast<volatile linuxCncConf_t*>(&rx_buf[rx_fill_idx]);
      for (int i = 0; i < STEPGENS; ++i) {
        conf_position_scale[i] = c->stepper_position_scale[i];
        conf_max_accel[i] = c->stepper_max_accel[i];
        conf_init_pos_mu[i] = c->stepper_init_position[i];
      }
      servo_period_s = c->servo_period_s;
      conf_pending_mask = (STEPGENS >= 32) ? 0xFFFFFFFFu : ((1u << STEPGENS) - 1u);
    }
    // keep cadence without publishing new state
    this->data_ready_callback();
    // rotate fill buffer anyway to avoid overwriting same slot repeatedly
    rx_fill_idx ^= 1u;
  }

  // RX re-armed from TX DMA TC to keep half-duplex cadence
}
