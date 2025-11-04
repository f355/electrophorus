#pragma once
#include <cstdint>

#include "hal.h"
#include "spi_protocol/spi_protocol.h"

class Stepgen {
 public:
  Stepgen() = default;
  int Init(int comp_id, const char *modname, const char *prefix, int i);
  void Update(double period_s, LinuxCncState *linuxcnc_state);
  void ApplyFeedback(const PruState *pru_state);
  void ResetPrevFeedback(const PruState *pru_state);

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

  Pins *pin_ = nullptr;
  Params *param_ = nullptr;

  int index_ = 0;
  double old_position_cmd_ = 0.0;
  int32_t prev_feedback_ = 0;

  double PositionControl(double period_s);
};
