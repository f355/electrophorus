
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

  MachineInit();

  // set the interrupt priorities
  NVIC_SetPriority(TIMER0_IRQn, 0);  // stepgen ticker
  NVIC_SetPriority(DMA_IRQn, 1);     // SpiComms DMA
  NVIC_SetPriority(EINT3_IRQn, 2);   // GPIO - PulseCounter, eStop
  NVIC_SetPriority(PendSV_IRQn, 3);  // RxListener

  const auto modules = MachineModules();

  printf("initializing stepgen ticker...\n");
  const auto stepgen_ticker = StepgenTicker::Instance();
  stepgen_ticker->RegisterModules(modules);
  stepgen_ticker->Start();
  printf("stepgen ticker initialized.\n");

  printf("initializing receive listener...\n");
  RxListener::Instance()->RegisterModules(modules);
  RxListener::Start();
  printf("receive listener initialized.\n");

  // take the initial input readings and write zeros to the outputs
  RxListener::HandleInterrupt();

  printf("initializing SPI...\n");
  const auto comms = SpiComms::Instance();
  comms->Start();
  printf("SPI initialized.\n");

  Watchdog::get_instance().start(2000);

  comms->Loop();
}
