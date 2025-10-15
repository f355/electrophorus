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

void machine_init() {
  // the beeper is obnoxious, shut it up first thing
  (new Pin(1, 14))->as_output()->set(false);
  // the addressable LED strip needs to be bit-banged at 800kHz and there's currently no support for that,
  // so for now let's just set it low and forget about it
  (new Pin(1, 15))->as_output()->set(false);
}

vector<Module*> machine_modules(SpiComms* comms) {
  const inputPin_t input_pins[INPUT_PINS] = INPUT_PIN_DESC;
  const outputPin_t output_pins[OUTPUT_PINS] = OUTPUT_PIN_DESC;

  return {
      // XYZA
      new Stepgen(0, new Pin(1, 28), (new Pin(1, 29))->invert(), comms),
      new Stepgen(1, new Pin(1, 26), new Pin(1, 27), comms),
      new Stepgen(2, new Pin(1, 24), (new Pin(1, 25))->invert(), comms),
      new Stepgen(3, new Pin(1, 21), (new Pin(1, 23))->invert(), comms),

      // e-stop
      new EStop((new Pin(0, 20))->invert(), comms),

      new DigitalIns(INPUT_PINS, input_pins, comms),  //
      new DigitalOuts(OUTPUT_PINS, output_pins, comms),

      new PulseCounter(0, new Pin(2, 7), comms),  // spindle encoder feedback

      // PWMs
      new PWM(0, new Pin(2, 5), comms),  // spindle - keep this at output_var=0 for e-stop to kill it
      new PWM(1, new Pin(2, 1), comms),  // spindle fan
      new PWM(2, new Pin(2, 3), comms),  // power supply fan
      new PWM(3, new Pin(2, 2), comms),  // EXT port output

      // thermistor ADCs (converted to temperature on LinuxCNC side)
      new ADC(1, new Pin(1, 31), comms),  // spindle
      new ADC(2, new Pin(0, 26), comms),  // power supply
  };
}
