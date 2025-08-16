#include "machine_config.h"

#include "module.h"
#include "modules/adc/adc.h"
#include "modules/gpio/digital_ins.h"
#include "modules/gpio/digital_outs.h"
#include "modules/gpio/pulse_counter.h"
#include "modules/pwm/pwm.h"
#include "modules/stepgen/stepgen.h"

// Carvera Air CA1 configuration

vector<PruThread*> configure_threads(const SpiComms* comms) {
  // the beeper is obnoxious, shut it up first thing
  (new Pin(1, 14))->as_output()->set(false);
  // the addressable LED strip needs to be bit-banged at 800kHz and there's currently no support for that,
  // so for now let's just set it low and forget about it
  (new Pin(1, 15))->as_output()->set(false);

  printf("\ncreating threads for CA1\n");

  // Args: timer number, frequency, IRQ priority
  auto base_thread = new PruThread(0, BASE_THREAD_FREQUENCY, 2);
  printf("created base thread\n");

  const std::vector<Module*> base_modules = {
      // Axis step generators
      new Stepgen(0, new Pin(1, 28), (new Pin(1, 29))->invert(), base_thread->frequency, comms->rx_data,
                  comms->tx_data),  // X
      new Stepgen(1, new Pin(1, 26), new Pin(1, 27), base_thread->frequency, comms->rx_data,
                  comms->tx_data),  // Y
      new Stepgen(2, new Pin(1, 24), (new Pin(1, 25))->invert(), base_thread->frequency, comms->rx_data,
                  comms->tx_data),  // Z
      new Stepgen(3, new Pin(1, 21), (new Pin(1, 23))->invert(), base_thread->frequency, comms->rx_data,
                  comms->tx_data)  // A
  };
  printf("created base modules\n");

  for (const auto m : base_modules) base_thread->register_module(m);
  printf("registered base modules\n");

  auto servo_thread = new PruThread(1, SERVO_THREAD_FREQUENCY, 3);
  printf("created servo thread\n");

  const inputPin_t input_pins[INPUT_PINS] = INPUT_PIN_DESC;
  const outputPin_t output_pins[OUTPUT_PINS] = OUTPUT_PIN_DESC;

  const std::vector<Module*> servo_modules = {
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

  printf("created servo modules\n");

  for (const auto m : servo_modules) servo_thread->register_module(m);
  printf("registered servo modules\n");

  return {base_thread, servo_thread};
}
