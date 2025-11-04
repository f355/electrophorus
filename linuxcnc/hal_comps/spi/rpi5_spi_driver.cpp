#include "rpi5_spi_driver.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <memory>

#include "rtapi.h"

// RP1 BAR1 (physical) and sizes
constexpr unsigned long long RP1_BAR1 = 0x1F00000000ULL;
constexpr unsigned long RP1_BAR1_LEN = 0x00400000UL;
constexpr unsigned long RP1_SPI0_BASE = 0x00050000UL;

// DW SPI register offsets
constexpr uint32_t DW_SPI_CTRLR0 = 0x00;
constexpr uint32_t DW_SPI_SSIENR = 0x08;
constexpr uint32_t DW_SPI_SER = 0x10;
constexpr uint32_t DW_SPI_BAUDR = 0x14;
constexpr uint32_t DW_SPI_RXFLR = 0x24;
constexpr uint32_t DW_SPI_IMR = 0x2c;
constexpr uint32_t DW_SPI_ICR = 0x48;
constexpr uint32_t DW_SPI_DR = 0x60;

// GPIO: IO_BANK0 and PADS_BANK0 (only for basic setup of 8..11)
constexpr unsigned long RP1_IO_BANK0 = 0x000D0000UL;
constexpr unsigned long RP1_PADS_BANK0 = 0x000F0000UL;
constexpr uint32_t RP1_GPIO_CTRL_FSEL_MASK = 0x1FU << 0;
constexpr uint32_t RP1_PADS_OD_SET = 1U << 7;
constexpr uint32_t RP1_PADS_IE_SET = 1U << 6;
constexpr uint32_t RP1_PADS_PUE_SET = 1U << 3;
constexpr uint32_t RP1_PADS_PDE_SET = 1U << 2;

constexpr uint32_t RP1_SPI_REF_CLK_HZ = 200000000U;

int Rpi5SpiDriver::init(int frequency_hz) {
  if (!compat_contains_any({
          "raspberrypi,5",
          "raspberrypi,5cm",
          "raspberrypi,5-model-b",
      }))
    return 0;  // not supported on this machine

  mem_fd = rtapi_open_as_root("/dev/mem", O_RDWR | O_SYNC);
  if (mem_fd < 0) return -errno;

  bar =
      static_cast<volatile uint8_t*>(mmap(nullptr, RP1_BAR1_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, RP1_BAR1));
  if (bar == MAP_FAILED) {
    const int e = -errno;
    close(mem_fd);
    mem_fd = -1;
    return e;
  }

  spi = bar + RP1_SPI0_BASE;

  // GPIO 8..11 to ALT0, disable pulls
  volatile void* io = bar + RP1_IO_BANK0;
  volatile void* pads = bar + RP1_PADS_BANK0;
  for (uint32_t g = 8; g <= 11; ++g) {
    uint32_t ctrl = rd32(io, (g * 2u + 1u) * 4u);
    ctrl &= ~RP1_GPIO_CTRL_FSEL_MASK;  // ALT0
    wr32(io, (g * 2u + 1u) * 4u, ctrl);
    uint32_t pad = rd32(pads, 4u + g * 4u);
    pad |= RP1_PADS_IE_SET;
    pad &= ~RP1_PADS_OD_SET;
    pad &= ~(RP1_PADS_PUE_SET | RP1_PADS_PDE_SET);
    wr32(pads, 4u + g * 4u, pad);
  }

  // Configure DW SPI controller: mode 1, 8-bit, baud
  wr32(spi, DW_SPI_SSIENR, 0);
  const uint32_t imr = rd32(spi, DW_SPI_IMR);
  wr32(spi, DW_SPI_IMR, imr & ~0xffu);
  (void)rd32(spi, DW_SPI_ICR);
  wr32(spi, DW_SPI_SER, 0);
  wr32(spi, DW_SPI_SSIENR, 1);
  wr32(spi, DW_SPI_SSIENR, 0);

  uint32_t cr0 = 0;
  cr0 |= (7u << 0);  // 8-bit
  cr0 |= (1u << 6);  // SPI mode 1 (CPHA=1, CPOL=0)
  wr32(spi, DW_SPI_CTRLR0, cr0);

  if (frequency_hz <= 0 || static_cast<uint32_t>(frequency_hz) > RP1_SPI_REF_CLK_HZ) frequency_hz = 5'000'000;
  const auto target = static_cast<uint32_t>(frequency_hz);
  uint32_t div = (RP1_SPI_REF_CLK_HZ + target - 1u) / target;  // ceil
  div = (div + 1u) & ~1u;                                      // make even
  if (div < 2u) div = 2u;
  wr32(spi, DW_SPI_BAUDR, div);

  wr32(spi, DW_SPI_SSIENR, 1);
  return 1;
}

void Rpi5SpiDriver::xfer(uint8_t* rx, const uint8_t* tx, size_t len) {
  if (!spi) return;
  wr32(spi, DW_SPI_SER, 1u << 0);  // assert CS0
  for (size_t i = 0; i < len; ++i) {
    wr32(spi, DW_SPI_DR, tx[i]);
    while (rd32(spi, DW_SPI_RXFLR) == 0) {
    }
    rx[i] = static_cast<uint8_t>(rd32(spi, DW_SPI_DR));
  }
  wr32(spi, DW_SPI_SER, 0);  // deassert CS0
}
