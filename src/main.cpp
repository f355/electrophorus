
/*
Electrophorus (originally Remora) PRU firmware for LinuxCNC
Copyright (C) 2021  Scott Alford (scotta)
Copyright (C) 2025 Konstantin Tcepliaev <f355@f355.org>

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License version 2
of the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include <chrono>

#include "machine_config.h"
#include "mbed.h"
#include "rx_listener.h"
#include "serial_comms.h"
#include "stepgen_ticker.h"

static constexpr auto COMMS_TIMEOUT = std::chrono::milliseconds(25);

static constexpr auto IDLE_REFRESH_INTERVAL = std::chrono::milliseconds(100);

enum State { ST_IDLE = 0, ST_RUNNING };

[[noreturn]] int main() {
  printf("\nelectrophorus booting up...\n");

  machine_init();

  SerialComms* comms = new SerialComms();
  const auto modules = machine_modules(comms);

  printf("stepgen ticker: start\n");
  const auto stepgen_ticker = StepgenTicker::instance();
  stepgen_ticker->register_modules(modules);
  stepgen_ticker->start();

  printf("rx listener: start\n");
  RxListener::instance()->register_modules(modules);
  RxListener::start();

  Watchdog::get_instance().start(2000);

  Timer comms_timer;
  comms_timer.start();
  auto last_ready = comms_timer.elapsed_time();
  auto last_idle_refresh = last_ready - IDLE_REFRESH_INTERVAL;
  auto last_read_token_log = last_ready;
  State current_state = ST_IDLE;
  State prev_state = ST_RUNNING;
  bool prev_e_stop_active = comms->e_stop_active;

  while (true) {
    Watchdog::get_instance().kick();

    if (comms->e_stop_active != prev_e_stop_active) {
      if (comms->e_stop_active) {
        printf("e-stop pressed, machine halted!\n");
      } else {
        printf("e-stop released, resuming operation...\n");
      }
      prev_e_stop_active = comms->e_stop_active;
    }

    switch (current_state) {
      case ST_IDLE:
        if (current_state != prev_state) {
          printf("waiting for LinuxCNC...\n");
        }
        prev_state = current_state;

        // refresh IO during idle
        if ((comms_timer.elapsed_time() - last_idle_refresh) > IDLE_REFRESH_INTERVAL) {
          RxListener::run();
          last_idle_refresh = comms_timer.elapsed_time();
        }

        if (comms->data_ready) {
          current_state = ST_RUNNING;
          last_ready = comms_timer.elapsed_time();
        }
        break;

      case ST_RUNNING:
        if (current_state != prev_state) {
          printf("running...\n");
        }
        prev_state = current_state;

        if (comms->data_ready) {
          last_ready = comms_timer.elapsed_time();
          comms->data_ready = false;
        }

        if ((comms_timer.elapsed_time() - last_ready) > COMMS_TIMEOUT) {
          printf("no communication from LinuxCNC (e-stop active?), resetting receive buffer...\n");
          // set the whole received buffer to 0
          for (volatile uint8_t& b : comms->get_linuxcnc_state()->buffer) b = 0;
          last_ready = comms_timer.elapsed_time();
          current_state = ST_IDLE;
        }
        break;
    }

    // once-a-second instrumentation: PRU_READ ok counter
    if ((comms_timer.elapsed_time() - last_read_token_log) > 1s) {
      printf("read_token_ok_count=%lu\n", (unsigned long)comms->read_token_ok_count);
      last_read_token_log = comms_timer.elapsed_time();
    }

    wait_us(1000);  // 1 ms idle
  }
}
