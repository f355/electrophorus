#include "machine_config.h"

#include "module.h"
#include "modules/adc/adc.h"
#include "modules/e_stop/e_stop.h"
#include "modules/gpio/digital_ins.h"
#include "modules/gpio/digital_outs.h"
#include "modules/gpio/pulse_counter.h"
#include "modules/pwm/pwm.h"
#include "modules/stepgen/stepgen.h"
#include "stepgen_ticker.h"

// Carvera Air CA1 configuration

void machine_init() {
  // using direct register poking in case pin outputs are disabled for debugging safety via EPHO_NO_HW_IO
  // the beeper is obnoxious, shut it up first thing
  // (new Pin(1, 14))->as_output()->set(false);
  gpio_ports[1]->FIODIR |= 1 << 14;
  gpio_ports[1]->FIOCLR = 1 << 14;
  // the addressable LED strip needs to be bit-banged at 800kHz and there's currently no support for that,
  // so for now let's just set it low and forget about it
  //(new Pin(1, 15))->as_output()->set(false);
  gpio_ports[1]->FIODIR |= 1 << 15;
  gpio_ports[1]->FIOCLR = 1 << 15;
}

vector<Module*> machine_modules(SerialComms* comms) {
  const inputPin_t input_pins[INPUT_PINS] = INPUT_PIN_DESC;
  const outputPin_t output_pins[OUTPUT_PINS] = OUTPUT_PIN_DESC;

  return {
      // XYZA
      new Stepgen(0, new Pin(1, 28), (new Pin(1, 29))->invert(), STEPGEN_FREQUENCY, comms),
      new Stepgen(1, new Pin(1, 26), new Pin(1, 27), STEPGEN_FREQUENCY, comms),
      new Stepgen(2, new Pin(1, 24), (new Pin(1, 25))->invert(), STEPGEN_FREQUENCY, comms),
      new Stepgen(3, new Pin(1, 21), (new Pin(1, 23))->invert(), STEPGEN_FREQUENCY, comms),

      // e-stop
      new EStop((new Pin(0, 20))->invert(), comms),

      new DigitalIns(INPUT_PINS, input_pins, comms),  //
      new DigitalOuts(OUTPUT_PINS, output_pins, comms),

      new PulseCounter(0, new Pin(2, 7), comms),  // spindle encoder feedback

      // PWMs
      // on LPC1768, the period is shared among all PWMs,
      // so don't try setting it to different values - the last one wins.
      // many bothans died to bring us this information.
      new PWM(0, new Pin(2, 5), 10000, comms),  // spindle - keep this at output_var=0 for e-stop to kill it
      new PWM(1, new Pin(2, 1), 10000, comms),  // spindle fan
      new PWM(2, new Pin(2, 3), 10000, comms),  // power supply fan
      new PWM(3, new Pin(2, 2), 10000, comms),  // EXT port output

      // thermistor ADCs (converted to temperature on LinuxCNC side)
      new ADC(1, new Pin(1, 31), comms),  // spindle
      new ADC(2, new Pin(0, 26), comms),  // power supply
  };
}
