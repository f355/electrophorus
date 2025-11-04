#pragma once
#include <cstdint>

#include "../../../src/spi_protocol.h"
#include "hal.h"

class Stepgen {
 public:
  Stepgen() = default;
  int init(int comp_id, const char *modname, const char *prefix, int i);
  void update(double period_s, LinuxCncState *linuxcnc_state);
  void apply_feedback(const PruState *pru_state);
  void reset_prev_feedback(const PruState *pru_state);

 private:
  struct Pins {
    hal_float_t *position_cmd;
    hal_float_t *velocity_cmd;
    hal_float_t *position_fb;
    hal_float_t *velocity_fb;
    hal_bit_t *enable;
    hal_bit_t *control_type;
    hal_bit_t *position_reset;
  };

  struct Params {
    hal_float_t position_scale;
    hal_float_t maxvel;
    hal_float_t maxaccel;
  };

  Pins *pin = nullptr;
  Params *param = nullptr;

  int index = 0;
  double old_position_cmd = 0.0;
  int32_t prev_feedback = 0;

  double position_control(double period_s);
};
