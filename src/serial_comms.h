#ifndef SERIAL_COMMS_H
#define SERIAL_COMMS_H

#include "LPC17xx.h"
#include "MODDMA.h"
#include "mbed.h"
#include "protocol_definitions.h"

class SerialComms final {
  // UART0 on LPC1768: TX=P0_2, RX=P0_3 (configured via UnbufferedSerial)
  mbed::UnbufferedSerial* serial;

  // Underlying UART base (can be reassigned to UART3, etc.)
  volatile LPC_UART_TypeDef* uart = nullptr;

  // Double buffers (published and fill for RX/TX)
  volatile linuxCncState_t rx_buf[2]{};
  volatile pruState_t tx_buf[2]{};

  // Public pointers to current buffers for const getter compatibility
  volatile linuxCncState_t* linuxcnc_state = nullptr;
  volatile pruState_t* pru_state = nullptr;

  // Pending configuration from host (per-axis)
  volatile float conf_position_scale[STEPGENS] = {};
  volatile float conf_max_accel[STEPGENS] = {};
  volatile float conf_init_pos_mu[STEPGENS] = {};
  volatile float servo_period_s = 0.00125f; // default 800 Hz
  volatile uint32_t conf_pending_mask = 0;

  // Indices and counters
  volatile uint8_t rx_fill_idx = 0;
  volatile uint8_t rx_ready_idx = 0;
  volatile uint8_t tx_fill_idx = 0;
  volatile uint8_t tx_send_idx = 0;

  volatile bool tx_in_progress = false;  // true while a 62-byte TX DMA is active

  // DMA engine and configs
  MODDMA dma;
  MODDMA_Config tx_dma_cfg;
  MODDMA_Config rx_dma_cfg;

  // Same-tick support
  enum class RxPhase : uint8_t { ExpectRead, ExpectCmd };
  volatile RxPhase rx_phase = RxPhase::ExpectCmd;
  volatile uint32_t read_token_storage = 0;
  bool same_tick_mode = false;

  // Helpers
  void on_tx_dma_tc();
  void on_rx_dma_tc();
  void start_rx_dma_for_fill();
  void start_rx_dma_read_token();

 public:
  SerialComms();
  ~SerialComms() = default;

  // Event flags used by modules
  volatile bool e_stop_active = false;
  volatile bool data_ready = false;

  void data_ready_callback() {
    data_ready = true;
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
  }

  // Expose current buffers to other modules/ISRs
  [[nodiscard]] volatile linuxCncState_t* get_linuxcnc_state() const { return linuxcnc_state; }
  [[nodiscard]] volatile pruState_t* get_pru_state() const { return pru_state; }

  // Consume pending per-axis configuration (returns true if one was pending for this axis)
  [[nodiscard]] float get_servo_period_s() const { return servo_period_s; }

  bool take_stepgen_conf(int axis, float* pos_scale, float* maxaccel, float* init_pos_mu);

  volatile uint32_t bad_header_count = 0;
  volatile uint32_t read_token_ok_count = 0;
  volatile uint32_t tx_frames = 0;
  volatile uint32_t rx_frames = 0;
};

#endif  // SERIAL_COMMS_H
