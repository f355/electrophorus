#pragma once
#include <cstdint>
#include <memory>

#include "../../src/spi_protocol.h"
#include "hal.h"
#include "rtapi.h"
#include "spi/spi_driver.hpp"
#include "stepgen/stepgen.hpp"

class Electrophorus {
 public:
  Electrophorus() = default;
  int init();
  void updateFreq(long period_ns);
  void write(long period_ns);
  void read(long period_ns);

 private:
  void spi_transfer() { spi->xfer(pru_state.buffer, linuxcnc_state.buffer, SPI_BUF_SIZE); }

  [[nodiscard]] bool pin_err(const int retval) const {
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin export failed err=%d\n", modname, retval);
      hal_exit(comp_id);
      return true;
    }
    return false;
  }
  int comp_id = -1;
  const char *modname = "electrophorus";
  const char *prefix = "carvera";

  struct Pins {
    hal_bit_t *spi_enable;
    hal_bit_t *spi_reset;
    hal_bit_t *spi_status;
    hal_float_t *output_vars[OUTPUT_VARS];
    hal_float_t *input_vars[INPUT_VARS];
    hal_bit_t *outputs[OUTPUT_PINS];
    hal_bit_t *inputs[INPUT_PINS * 2];
  };
  Pins *pin = nullptr;

  std::unique_ptr<SpiDriver> spi;

  linuxCncState_t linuxcnc_state{};
  pruState_t pru_state{};
  bool spi_reset_old = false;
  uint32_t last_packet_seen = 0;
  Stepgen stepgens[STEPGENS]{};
};
