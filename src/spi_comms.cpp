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
  spi_set_rx_driven();
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
  // Burst TX first
  while (spi_writeable() && bytes_transmitted < sizeof(pruState_t)) {
    if (constexpr size_t crc_pos = offsetof(pruState_t, crc32); bytes_transmitted < crc_pos) {
      EphoCRC32::update(crc, tx_ptr[bytes_transmitted]);
    } else if (bytes_transmitted == crc_pos) {
      // finalize once when we reach the CRC field, store directly in the struct
      const_cast<pruState_t*>(this->pru_back)->crc32 = EphoCRC32::finalize(crc);
    }
    spi_write(tx_ptr[bytes_transmitted]);
    bytes_transmitted++;
  }

  // Then burst RX drain
  while (spi_readable() && bytes_received < sizeof(pruState_t)) {
    (void)spi_read();
    bytes_received++;
  }

  // If TX finished, disable TXIM, drain remaining RX bytes immediately, and preload next command response
  if (bytes_transmitted >= sizeof(pruState_t) && current_cmd == PruCommand::Read) {
    spi_set_rx_driven();
    while (bytes_received < sizeof(pruState_t)) {
      if (spi_readable()) {
        (void)spi_read();
        bytes_received++;
      }
    }
    current_cmd = PruCommand::None;
    preload_cmd_response();
  }
}

inline void SpiComms::try_read_payload_byte() {
  if (!spi_readable()) return;
  const auto b = spi_read();
  if (bytes_received < offsetof(linuxCncState_t, crc32)) {
    EphoCRC32::update(crc, b);
  }
  *rx_ptr++ = b;
  bytes_received++;
}

inline void SpiComms::receive_write_payload() {
  // Mirror transmit_read_response structure: TX-driven loop with opportunistic RX drain
  while (spi_writeable() && bytes_transmitted < sizeof(linuxCncState_t)) {
    // For WRITE, we transmit zeros while host clocks data in
    spi_write(0);
    bytes_transmitted++;

    // Drain one RX byte if available
    if (bytes_received < sizeof(linuxCncState_t)) try_read_payload_byte();
  }

  // If TX finished, disable TXIM, drain remaining RX bytes immediately, and preload next command response
  if (bytes_transmitted >= sizeof(linuxCncState_t) && current_cmd == PruCommand::Write) {
    spi_set_rx_driven();

    // Drain any remaining RX bytes (tail) before validating CRC
    while (bytes_received < sizeof(linuxCncState_t)) try_read_payload_byte();

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
    preload_cmd_response();
  }
}

inline void SpiComms::discard_payload() {
  // TX zeros to satisfy clocks and drain RX
  while (spi_writeable() && bytes_transmitted < discard_size) {
    spi_write(0);
    bytes_transmitted++;
  }
  while (spi_readable() && bytes_received < discard_size) {
    (void)spi_read();
    bytes_received++;
  }
  if (bytes_received >= discard_size) {
    preload_cmd_response();
    current_cmd = PruCommand::None;
    spi_set_rx_driven();
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
  switch (cmd) {
    case PRU_READ: {
      data_ready = true;
      const volatile auto tmp = this->pru_state;
      this->pru_state = this->pru_back;
      this->pru_back = tmp;

      current_cmd = PruCommand::Read;
      tx_ptr = reinterpret_cast<const uint8_t*>(const_cast<const pruState_t*>(this->pru_back));
      bytes_transmitted = 0;
      bytes_received = 0;
      EphoCRC32::init(crc);
      // Prefill immediately
      transmit_read_response();
      break;
    }
    case PRU_WRITE: {
      data_ready = true;
      current_cmd = PruCommand::Write;
      rx_ptr = reinterpret_cast<uint8_t*>(const_cast<linuxCncState_t*>(this->linuxcnc_back));
      bytes_received = 0;
      bytes_transmitted = 0;
      EphoCRC32::init(crc);
      receive_write_payload();
      break;
    }
    default: {
      reject_count++;
      if (reject_count > 5) spi_error = true;
      discard_size = (current_cmd == PruCommand::Read) ? sizeof(linuxCncState_t) : sizeof(pruState_t);
      current_cmd = PruCommand::Invalid;
      bytes_received = 0;
      bytes_transmitted = 0;
      discard_payload();
    }
  }
  // Common: enable TX-driven transfer after setup
  spi_set_tx_driven();
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
