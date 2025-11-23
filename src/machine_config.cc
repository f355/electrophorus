#include "machine_config.h"

#include "modules/adc/adc.h"
#include "modules/e_stop/e_stop.h"
#include "modules/gpio/digital_ins.h"
#include "modules/gpio/digital_outs.h"
#include "modules/gpio/pulse_counter.h"
#include "modules/module.h"
#include "modules/pwm/pwm.h"
#include "modules/stepgen/stepgen.h"
#include "stepgen_ticker.h"

// Carvera Air CA1 configuration

void MachineInit() {
  // the beeper is obnoxious, shut it up first thing
  (new Pin(1, 14))->AsOutput()->Set(false);
  // the addressable LED strip needs to be bit-banged at 800kHz and there's currently no support for that,
  // so for now let's just set it low and forget about it
  (new Pin(1, 15))->AsOutput()->Set(false);
}

vector<Module*> MachineModules() {
  return {
      new Stepgen(0, new Pin(1, 28), (new Pin(1, 29))->Invert()),  // X
      new Stepgen(1, new Pin(1, 26), new Pin(1, 27)),              // Y
      new Stepgen(2, new Pin(1, 24), (new Pin(1, 25))->Invert()),  // Z
      new Stepgen(3, new Pin(1, 21), (new Pin(1, 23))->Invert()),  // A

      // e-stop
      new EStop((new Pin(0, 20))->Invert()),

      new DigitalIns(kNumInputPins, kInputPins),  //
      new DigitalOuts(kNumOutputPins, kOutputPins),

      new PulseCounter(0, new Pin(2, 7)),  // spindle encoder feedback

      // PWMs
      new PWM(1, 0, new Pin(2, 5)),  // spindle - keep this at output_var=0 for e-stop to kill it
      new PWM(2, 0, new Pin(2, 1)),  // spindle fan
      new PWM(3, 0, new Pin(2, 3)),  // power supply fan
      new PWM(4, 0, new Pin(2, 2)),  // EXT port output

      // thermistor ADCs (converted to temperature on LinuxCNC side)
      new ADC(1, new Pin(1, 31)),  // spindle
      new ADC(2, new Pin(0, 26)),  // power supply
  };
}
