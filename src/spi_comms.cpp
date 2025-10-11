#include "spi_comms.h"

#include "LPC17xx.h"
#include "machine_definitions.h"

#define MAX_SPI_DELAY 5

#include "pin.h"
static Pin dbg_pin(0, 8);

enum State { ST_IDLE = 0, ST_RUNNING, ST_RESET };

#include "epho_crc32.h"

SpiComms* SpiComms::s_instance = nullptr;

SpiComms::SpiComms() {
  // Initialize buffer pointers
  this->linuxcnc_state = &this->linuxcnc_state1;
  this->linuxcnc_back = &this->linuxcnc_state2;
  this->pru_state = &this->pru_state1;
  this->pru_back = &this->pru_state2;

  spi_init(&spi, SPI_MOSI, SPI_MISO, SPI_SCK, SPI_SSEL);
  spi_format(&spi, 8, 0, 1);      // 8-bit transfers, mode 0, on the receiving end
  spi_frequency(&spi, 10000000);  // this is still needed for some reason

  // Debug pin init
  dbg_pin.as_output();
  dbg_pin.set(true);

  s_instance = this;

  NVIC_SetPriority(SSP0_IRQn, SSP_IRQ_PRIORITY);
  NVIC_EnableIRQ(SSP0_IRQn);

  spi.spi->ICR = 1u << 0 | 1u << 1;  // clear RORIC/RTIC
  spi_rx_irq(true);
  spi_tx_irq(false);
  preload_cmd_response();
}

extern "C" void SSP0_IRQHandler(void) {
  if (SpiComms::s_instance) SpiComms::s_instance->ssp0_irq();
}

linuxCncState_t volatile* SpiComms::get_linuxcnc_state() const { return this->linuxcnc_state; }
pruState_t volatile* SpiComms::get_pru_state() const { return this->pru_state; }

inline void SpiComms::preload_cmd_response() const {
  constexpr uint32_t resp = PRU_DATA;
  for (size_t i = 0; i < 4; ++i) spi_write(reinterpret_cast<const uint8_t*>(&resp)[i]);
}

inline void SpiComms::transmit_read_response() {
  while (tx_remaining > 0 && spi_writeable()) {
    uint8_t out = 0;
    const size_t idx = sizeof(pruState_t) - tx_remaining;
    if (constexpr size_t crc_pos = offsetof(pruState_t, crc32); idx < crc_pos) {
      out = tx_ptr[idx];
      EphoCRC32::update(crc, out);
    } else {
      if (idx == crc_pos) {
        // finalize once when we reach the CRC field, store directly in the struct
        const_cast<pruState_t*>(this->pru_back)->crc32 = EphoCRC32::finalize(crc);
      }
      out = tx_ptr[idx];
    }
    spi_write(out);
    tx_remaining--;

    if (rx_remaining > 0 && spi_readable()) {
      (void)spi_read();
      rx_remaining--;
    }
  }

  // If TX finished, disable TXIM, drain remaining RX bytes immediately, and preload next command zeros
  if (tx_remaining == 0 && current_cmd == PruCommand::Read) {
    spi_tx_irq(false);
    while (rx_remaining > 0) {
      if (spi_readable()) {
        (void)spi_read();
        rx_remaining--;
      }
    }
    current_cmd = PruCommand::None;
    spi_rx_irq(true);
    preload_cmd_response();
  }
}

inline void SpiComms::try_read_payload_byte() {
  if (!spi_readable()) return;
  const auto b = spi_read();
  if (sizeof(linuxCncState_t) - rx_remaining < offsetof(linuxCncState_t, crc32)) {
    EphoCRC32::update(crc, b);
  }
  *rx_ptr++ = b;
  rx_remaining--;
}

inline void SpiComms::receive_write_payload() {
  // Mirror transmit_read_response structure: TX-driven loop with opportunistic RX drain
  while (tx_remaining > 0 && spi_writeable()) {
    // For WRITE, we transmit zeros while host clocks data in
    spi_write(0);
    tx_remaining--;

    // Drain one RX byte if available
    if (rx_remaining > 0) try_read_payload_byte();
  }

  // If TX finished, disable TXIM, drain remaining RX bytes immediately, and preload next command zeros
  if (tx_remaining == 0 && current_cmd == PruCommand::Write) {
    spi_tx_irq(false);

    // Drain any remaining RX bytes (tail) before validating CRC
    while (rx_remaining > 0) try_read_payload_byte();

    // Now the entire payload is received; verify and publish
    if (!this->e_stop_active) {
      if (this->linuxcnc_back->crc32 == EphoCRC32::finalize(crc)) {
        reject_count = 0;
        const volatile auto tmp = this->linuxcnc_state;
        this->linuxcnc_state = this->linuxcnc_back;
        this->linuxcnc_back = tmp;
        data_ready_callback();
      } else {
        reject_count++;
        if (reject_count > 5) spi_error = true;
      }
    }

    current_cmd = PruCommand::None;
    spi_rx_irq(true);
    preload_cmd_response();
  }
}

inline void SpiComms::discard_payload() {
  // TX zeros to satisfy clocks and drain RX
  while (tx_remaining > 0 && spi_writeable()) {
    spi_write(0);
    tx_remaining--;
  }
  while (rx_remaining > 0 && spi_readable()) {
    (void)spi_read();
    rx_remaining--;
  }
  if (rx_remaining == 0) {
    preload_cmd_response();
    current_cmd = PruCommand::None;
    spi_tx_irq(false);
    spi_rx_irq(true);
  }
}

inline void SpiComms::wait_for_command() {
  // Drain 4 command bytes
  size_t cmd_idx = 0;
  uint32_t cmd;
  if (!spi_readable()) return;  // don't block the ISR - RXIM will fire again when the command arrives
  while (cmd_idx < 4) {
    if (spi_readable()) {
      reinterpret_cast<uint8_t*>(&cmd)[cmd_idx++] = spi_read();
    }
  }
  // payload transfer is TX-driven
  spi_rx_irq(false);
  spi_tx_irq(true);
  switch (cmd) {
    case PRU_READ: {
      data_ready = true;
      const volatile auto tmp = this->pru_state;
      this->pru_state = this->pru_back;
      this->pru_back = tmp;
      __DSB();
      current_cmd = PruCommand::Read;
      tx_ptr = reinterpret_cast<volatile const uint8_t*>(this->pru_back);
      tx_remaining = sizeof(pruState_t);
      rx_ptr = nullptr;
      rx_remaining = sizeof(pruState_t);
      EphoCRC32::init(crc);
      // Prefill immediately
      transmit_read_response();
      break;
    }
    case PRU_WRITE: {
      data_ready = true;
      current_cmd = PruCommand::Write;
      rx_ptr = reinterpret_cast<volatile uint8_t*>(this->linuxcnc_back);
      rx_remaining = sizeof(linuxCncState_t);
      tx_ptr = nullptr;
      tx_remaining = sizeof(linuxCncState_t);
      EphoCRC32::init(crc);
      receive_write_payload();
      break;
    }
    default: {
      reject_count++;
      if (reject_count > 5) spi_error = true;
      const size_t discard_size = (current_cmd == PruCommand::Read) ? sizeof(linuxCncState_t) : sizeof(pruState_t);
      current_cmd = PruCommand::Invalid;
      rx_ptr = nullptr;
      rx_remaining = discard_size;
      tx_ptr = nullptr;
      tx_remaining = discard_size;
      discard_payload();
    }
  }
}

void SpiComms::ssp0_irq() {
  // Mode-driven service
  switch (current_cmd) {
    case PruCommand::Read:
      transmit_read_response();
      break;
    case PruCommand::Write:
      receive_write_payload();
      break;
    case PruCommand::Invalid:
      discard_payload();
      break;
    case PruCommand::None: {
      wait_for_command();
      break;
    }
  }
}

void SpiComms::data_ready_callback() {
  // trigger PendSV IRQ to signal that the data is ready
  SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

void SpiComms::loop() {
  uint8_t spi_delay = 0;
  State current_state = ST_IDLE;
  State prev_state = ST_RESET;
  bool prev_e_stop_active = this->e_stop_active;

  while (true) {
    Watchdog::get_instance().kick();

    if (this->e_stop_active != prev_e_stop_active) {
      if (this->e_stop_active) {
        printf("e-stop pressed, machine halted!\n");
      } else {
        printf("e-stop released, resuming operation...\n");
      }
      prev_e_stop_active = this->e_stop_active;
    }

    if (this->spi_error) {
      printf("SPI communication error, ignoring. If you see a lot of these, check your cabling.\n");
      this->spi_error = false;
    }

    switch (current_state) {
      case ST_IDLE:
        if (current_state != prev_state) {
          printf("waiting for LinuxCNC...\n");
        }
        prev_state = current_state;

        if (this->data_ready) {
          current_state = ST_RUNNING;
        }
        break;

      case ST_RUNNING:
        if (current_state != prev_state) {
          printf("running...\n");
        }
        prev_state = current_state;

        if (this->data_ready) {
          spi_delay = 0;
          this->data_ready = false;
        } else {
          spi_delay++;
        }

        if (spi_delay > MAX_SPI_DELAY) {
          printf("no communication from LinuxCNC, e-stop active?\n");
          spi_delay = 0;
          current_state = ST_RESET;
        }
        break;
      case ST_RESET:
        if (current_state != prev_state) {
          printf("resetting receive buffer...\n");
        }
        prev_state = current_state;

        // clear the whole LinuxCNC state buffer
        // can't memset volatile memory, so use a loop instead
        for (size_t i = 0; i < sizeof(linuxCncState_t); ++i)
          reinterpret_cast<volatile uint8_t*>(this->linuxcnc_state)[i] = 0;
        data_ready_callback();

        current_state = ST_IDLE;
    }

    wait_us(1000);
  }
}
