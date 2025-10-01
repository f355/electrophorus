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
  MODDMA_Config rx_dma_cfg[2];
  static constexpr MODDMA::CHANNELS RX_DMA_CHANNELS[2] = { MODDMA::Channel_1, MODDMA::Channel_2 };
  volatile uint8_t rx_dma_ch_idx = 0;

  // Unified RX state machine: accept PRU_READ (4B), PRU_WRITE/PRU_DATA/PRU_CONF (62B)
  enum class RxPhase : uint8_t { ExpectHeader, ExpectPayload };
  volatile RxPhase rx_phase = RxPhase::ExpectHeader;
  volatile uint32_t read_token_storage = 0; // header/temp storage
  volatile uint32_t current_header = 0;     // last parsed header

  // Helpers
  void on_tx_dma_tc();
  void on_rx_dma_tc();
  void start_rx_dma_read_token();      // 4B header
  void start_rx_dma_payload58();       // 58B payload following 4B header

 public:
  SerialComms();
  ~SerialComms() = default;

  // Expose counters for instrumentation in main loop
  volatile uint32_t bad_header_count = 0;
  volatile uint32_t read_token_ok_count = 0;
  volatile uint32_t tx_frames = 0;
  volatile uint32_t rx_frames = 0;
  volatile uint32_t tx_tc = 0;
  volatile uint32_t rx_header_tc = 0;
  volatile uint32_t rx_payload_tc = 0;
  volatile uint32_t last_header = 0;
  volatile uint32_t rx_lsr_err = 0;
  volatile uint32_t rx_lsr_oe = 0;

  // Debug/instrumentation for RX re-arm and DMA state
  volatile uint32_t header_rearm_calls = 0;
  volatile uint32_t header_prepare_calls = 0;
  volatile uint32_t header_enable_calls = 0;
  volatile uint32_t payload_prepare_calls = 0;
  volatile uint32_t payload_enable_calls = 0;
  volatile uint32_t dbg_rx_enabled = 0;   // DMACCConfig E bit for CH1
  volatile uint32_t dbg_rx_active = 0;    // GPDMA active state for CH1
  volatile uint32_t dbg_enbld_chns = 0;   // DMACEnbldChns snapshot
  volatile uint32_t dbg_lsr = 0;          // UART LSR snapshot
  volatile uint32_t tx_skipped_busy = 0;  // PRU_READ arrived but TX already in progress



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
};

#endif  // SERIAL_COMMS_H
