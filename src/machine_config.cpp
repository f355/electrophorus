#include "machine_config.h"

#include "irq_ticker.h"
#include "module.h"
#include "modules/adc/adc.h"
#include "modules/gpio/digital_ins.h"
#include "modules/gpio/digital_outs.h"
#include "modules/gpio/pulse_counter.h"
#include "modules/pwm/pwm.h"
#include "modules/stepgen/stepgen.h"

// Carvera Air CA1 configuration

void machine_init() {
  // the beeper is obnoxious, shut it up first thing
  (new Pin(1, 14))->as_output()->set(false);
  // the addressable LED strip needs to be bit-banged at 800kHz and there's currently no support for that,
  // so for now let's just set it low and forget about it
  (new Pin(1, 15))->as_output()->set(false);
}

vector<Module*> machine_base_modules(const SpiComms* comms) {
  // XYZA
  return {new Stepgen(0, new Pin(1, 28), (new Pin(1, 29))->invert(), BASE_FREQUENCY, comms->rx_data, comms->tx_data),
          new Stepgen(1, new Pin(1, 26), new Pin(1, 27), BASE_FREQUENCY, comms->rx_data, comms->tx_data),
          new Stepgen(2, new Pin(1, 24), (new Pin(1, 25))->invert(), BASE_FREQUENCY, comms->rx_data, comms->tx_data),
          new Stepgen(3, new Pin(1, 21), (new Pin(1, 23))->invert(), BASE_FREQUENCY, comms->rx_data, comms->tx_data)};
}

vector<Module*> machine_servo_modules(const SpiComms* comms) {
  const inputPin_t input_pins[INPUT_PINS] = INPUT_PIN_DESC;
  const outputPin_t output_pins[OUTPUT_PINS] = OUTPUT_PIN_DESC;

  return {
      new DigitalIns(INPUT_PINS, input_pins, comms->tx_data),  //
      new DigitalOuts(OUTPUT_PINS, output_pins, comms->rx_data),

      new PulseCounter(0, new Pin(2, 7), comms->tx_data),  // spindle encoder feedback

      // PWMs
      // on LPC1768, the period is shared among all PWMs,
      // so don't try setting it to different values - the last one wins.
      // many bothans died to bring us this information.
      new PWM(0, new Pin(2, 5), 10000, comms->rx_data),  // spindle
      new PWM(1, new Pin(2, 1), 10000, comms->rx_data),  // spindle fan
      new PWM(2, new Pin(2, 3), 10000, comms->rx_data),  // power supply fan
      new PWM(3, new Pin(2, 2), 10000, comms->rx_data),  // EXT port output

      // thermistor ADCs (converted to temperature on LinuxCNC side)
      new ADC(1, new Pin(1, 31), comms->tx_data),  // spindle
      new ADC(2, new Pin(0, 26), comms->tx_data),  // power supply
  };
}
