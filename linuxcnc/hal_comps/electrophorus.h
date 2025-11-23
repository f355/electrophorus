#pragma once
#include <cstdint>
#include <memory>

#include "../../src/spi_protocol.h"
#include "hal.h"
#include "rtapi.h"
#include "spi/spi_driver.h"
#include "stepgen/stepgen.h"

class Electrophorus {
 public:
  Electrophorus() = default;
  int Init();
  void UpdateFreq(long period_ns);
  void Write(long period_ns);
  void Read(long period_ns);

 private:
  void SpiTransfer() { spi_->Xfer(rx_buffer_.bytes, tx_buffer_.bytes, tx_buffer_.Size()); }

  [[nodiscard]] bool PinErr(const int retval) const {
    if (retval < 0) {
      rtapi_print_msg(RTAPI_MSG_ERR, "%s: ERROR: pin export failed err=%d\n", modname_, retval);
      hal_exit(comp_id_);
      return true;
    }
    return false;
  }
  int comp_id_ = -1;
  const char *modname_ = "electrophorus";
  const char *prefix_ = "carvera";

  struct Pins {
    hal_bit_t *spi_enable;
    hal_bit_t *spi_reset;
    hal_bit_t *spi_status;
    hal_float_t *output_vars[kNumOutputVars];
    hal_float_t *input_vars[kNumInputVars];
    hal_bit_t *outputs[kNumOutputPins];
    hal_bit_t *inputs[kNumInputPins * 2];
  };
  Pins *pin_ = nullptr;

  std::unique_ptr<SpiDriver> spi_;

  SpiBuffer<LinuxCncState> tx_buffer_;
  SpiBuffer<PruState> rx_buffer_;
  LinuxCncState *linuxcnc_state_ = &tx_buffer_.AsStruct();
  PruState *pru_state_ = &rx_buffer_.AsStruct();

  bool spi_reset_old_ = false;
  uint32_t last_packet_seen_ = 0;
  Stepgen stepgens_[kNumStepgens]{};
};
