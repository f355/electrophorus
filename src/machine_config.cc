#include "machine_config.h"

#include <vector>

#include "modules/e_stop/e_stop.h"
#include "modules/gpio/digital_ins.h"
#include "modules/gpio/digital_outs.h"
#include "modules/module.h"
#include "modules/pwm/pwm.h"
#include "modules/stepgen/stepgen.h"
#include "pin.h"
#include "spi_protocol/machine_definitions.h"

// Proxxon PD 250/E lathe using Carvera C1 board

void MachineInit() {
  // Enable 24V rail for stepper drivers
  (new Pin(0, 10))->AsOutput()->Set(true);
}

vector<Module*> MachineModules() {
  return {
      // X = cross slide (C1 A-axis driver, TMC2209)
      new Stepgen(0, new Pin(1, 18), new Pin(1, 20)),
      // Z = carriage (C1 B-axis driver, TMC2209)
      new Stepgen(1, new Pin(1, 21), new Pin(1, 23)),

      new EStop((new Pin(0, 26))->Invert()),

      new DigitalIns(kNumInputPins, kInputPins),
      new DigitalOuts(kNumOutputPins, kOutputPins),

      // Spindle PWM (WS55-220 speed control via RC filter)
      new PWM(1, 0, new Pin(2, 5)),
  };
}
