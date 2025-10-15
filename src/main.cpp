
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

#include "machine_config.h"
#include "mbed.h"
#include "rx_listener.h"
#include "spi_comms.h"
#include "stepgen_ticker.h"

[[noreturn]] int main() {
  printf("\nelectrophorus booting up...\n");

  machine_init();

  printf("initializing SPI...\n");
  const auto comms = new SpiComms();
  printf("SPI initialized.\n");

  const auto modules = machine_modules(comms);

  printf("initializing stepgen ticker...\n");
  const auto stepgen_ticker = StepgenTicker::instance();
  stepgen_ticker->register_modules(modules);
  stepgen_ticker->start();
  printf("stepgen ticker initialized.\n");

  printf("initializing receive listener...\n");
  RxListener::instance()->register_modules(modules);
  RxListener::start();
  printf("receive listener initialized.\n");

  // take the initial input readings and write zeros to the outputs
  RxListener::handle_interrupt();

  Watchdog::get_instance().start(2000);

  comms->loop();
}
