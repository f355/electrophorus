#include "machine_config.h"

#include "modules/e_stop/e_stop.h"
#include "modules/gpio/digital_ins.h"
#include "modules/gpio/digital_outs.h"
#include "modules/module.h"
#include "modules/pwm/pwm.h"
#include "modules/quadrature_encoder/quadrature_encoder.h"
#include "modules/stepgen/stepgen.h"
#include "pin.h"

// Proxxon PD 250/E lathe using Carvera C1 board

const SpiConfig& MachineSpiConfig() {
  static constexpr SpiConfig config = {
      .mosi = P0_9,
      .miso = P0_8,
      .sck = P0_7,
      .ssel = P0_6,
      .ssp = LPC_SSP1,
      .dma_conn_tx = 2,  // SSP1 TX (UM10360 Table 544)
      .dma_conn_rx = 3,  // SSP1 RX
  };
  return config;
}

void MachineInit() {
  // Enable 24V rail for stepper drivers
  (new Pin(0, 10))->AsOutput()->Set(true);
}

vector<Module*> MachineModules() {
  return {// X = cross slide (C1 A-axis driver, TMC2209)
          new Stepgen(0, new Pin(1, 18), new Pin(1, 26)),
          // Z = carriage (C1 B-axis driver, TMC2209)
          new Stepgen(1, new Pin(1, 21), new Pin(1, 25)),

          new EStop((new Pin(0, 26))->Invert()),

          new DigitalIns(kNumInputPins, kInputPins), new DigitalOuts(kNumOutputPins, kOutputPins),

          // Spindle PWM (WS55-220 speed control via RC filter)
          new PWM(1, 0, new Pin(2, 5)),

          new QuadratureEncoder(0, 1, false, 10000)};
}
