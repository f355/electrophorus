/*
 RP1 SPI driver for Raspberry Pi 5 (SPI0, CS0, mode 1, 8-bit)

 This file is part of Electrophorus.

 Copyright (C) 2025 Konstantin Tcepliaev <f355@f355.org>

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

// RP1 BAR1 (physical) and sizes
#define RP1_BAR1 0x1F00000000ULL
#define RP1_BAR1_LEN 0x00400000UL

// SPI0 base offset within RP1 BAR1
#define RP1_SPI0_BASE 0x00050000UL

// DW SPI register offsets used
#define DW_SPI_CTRLR0 0x00
#define DW_SPI_SSIENR 0x08
#define DW_SPI_SER 0x10
#define DW_SPI_BAUDR 0x14
#define DW_SPI_RXFLR 0x24
#define DW_SPI_IMR 0x2c
#define DW_SPI_ICR 0x48
#define DW_SPI_DR 0x60

// GPIO (RP1) offsets and bitfields used (bank 0, GPIO 8..11)
#define RP1_IO_BANK0_OFFSET 0x000D0000UL
#define RP1_PADS_BANK0_OFFSET 0x000F0000UL
#define RP1_GPIO_CTRL_FSEL_LSB 0
#define RP1_GPIO_CTRL_FSEL_MASK (0x1FU << RP1_GPIO_CTRL_FSEL_LSB)
#define RP1_PADS_OD_SET (1U << 7)
#define RP1_PADS_IE_SET (1U << 6)
#define RP1_PADS_PUE_SET (1U << 3)
#define RP1_PADS_PDE_SET (1U << 2)

// 200 MHz reference for BAUDR divider
#define RP1_SPI_REF_CLK_HZ 200000000U

static uint32_t RP1_GPIO_IO_REG_CTRL_OFFSET(const uint32_t offset) { return (offset * 2U + 1U) * sizeof(uint32_t); }
static uint32_t RP1_GPIO_PADS_REG_OFFSET(const uint32_t offset) { return sizeof(uint32_t) + offset * sizeof(uint32_t); }

// Globals
static volatile uint8_t *rpi5_rp1_bar;   // mapped BAR1
static volatile uint8_t *rpi5_spi_base;  // SPI0 regs base
static int rpi5_mem_fd = -1;

// Helpers to read/write 32-bit regs
static uint32_t rpi5_rd32(volatile void *base, const uint32_t off) {
  return *(volatile uint32_t *)((volatile uint8_t *)base + off);
}
static inline void rpi5_wr32(volatile void *base, const uint32_t off, const uint32_t v) {
  *(volatile uint32_t *)((volatile uint8_t *)base + off) = v;
}

// Detect Raspberry Pi 5 via device-tree compatible
static int is_rpi5(void) {
  const int fd = open("/proc/device-tree/compatible", O_RDONLY);
  if (fd < 0) return 0;
  char buf[1024];
  const ssize_t n = read(fd, buf, sizeof(buf));
  close(fd);
  if (n <= 0) return 0;
  // compatible is NUL-separated strings
  const char *models[] = {"raspberrypi,5", "raspberrypi,5cm", "raspberrypi,5-model-b"};
  for (size_t i = 0; i < sizeof(models) / sizeof(models[0]); i++) {
    if (memmem(buf, (size_t)n, models[i], strlen(models[i]))) return 1;
  }
  return 0;
}

// Set GPIO 8..11 to ALT0 (SPI0 CS0/MISO/MOSI/SCLK), no pulls
static void setup_spi0_pins(void) {
  volatile void *io_b0 = rpi5_rp1_bar + RP1_IO_BANK0_OFFSET;
  volatile void *pads_b0 = rpi5_rp1_bar + RP1_PADS_BANK0_OFFSET;
  for (uint32_t gpio = 8; gpio <= 11; ++gpio) {
    const uint32_t off = gpio;  // bank0
    // CTRL: set FSEL = ALT0 (0)
    uint32_t ctrl = rpi5_rd32(io_b0, RP1_GPIO_IO_REG_CTRL_OFFSET(off));
    ctrl &= ~RP1_GPIO_CTRL_FSEL_MASK;
    // ALT0 is 0 -> nothing to OR in
    rpi5_wr32(io_b0, RP1_GPIO_IO_REG_CTRL_OFFSET(off), ctrl);
    // PADS: enable input, disable OD, no pulls
    uint32_t pad = rpi5_rd32(pads_b0, RP1_GPIO_PADS_REG_OFFSET(off));
    pad |= RP1_PADS_IE_SET;                         // input enable
    pad &= ~RP1_PADS_OD_SET;                        // clear open-drain -> enable peripheral output
    pad &= ~(RP1_PADS_PUE_SET | RP1_PADS_PDE_SET);  // pulls off
    rpi5_wr32(pads_b0, RP1_GPIO_PADS_REG_OFFSET(off), pad);
  }
}

// Configure SPI0 for mode 1, 8-bit, TR, CS0 idle high (inactive), set baud divider
static void setup_spi0_ctrl(int frequency_hz) {
  rpi5_wr32(rpi5_spi_base, DW_SPI_SSIENR, 0);
  const uint32_t imr = rpi5_rd32(rpi5_spi_base, DW_SPI_IMR);
  rpi5_wr32(rpi5_spi_base, DW_SPI_IMR, imr & ~0xffu);
  (void)rpi5_rd32(rpi5_spi_base, DW_SPI_ICR);
  rpi5_wr32(rpi5_spi_base, DW_SPI_SER, 0);
  rpi5_wr32(rpi5_spi_base, DW_SPI_SSIENR, 1);

  rpi5_wr32(rpi5_spi_base, DW_SPI_SSIENR, 0);

  uint32_t cr0 = 0;
  cr0 |= (7u << 0);  // DFS=8
  cr0 |= (1u << 6);  // CPHA
  rpi5_wr32(rpi5_spi_base, DW_SPI_CTRLR0, cr0);

  if (frequency_hz <= 0 || frequency_hz > (int)RP1_SPI_REF_CLK_HZ) frequency_hz = 5000000;
  uint32_t clk_div = ((RP1_SPI_REF_CLK_HZ + (uint32_t)frequency_hz - 1u) / (uint32_t)frequency_hz);
  clk_div = (clk_div + 1u) & 0xfffeu;
  if (clk_div == 0) clk_div = 2;
  rpi5_wr32(rpi5_spi_base, DW_SPI_BAUDR, clk_div);

  rpi5_wr32(rpi5_spi_base, DW_SPI_SSIENR, 1);
}

int rpi5_spi_init(const int frequency_hz) {
  if (!is_rpi5()) return 0;  // not RPi 5

  rpi5_mem_fd = rtapi_open_as_root("/dev/mem", O_RDWR | O_SYNC);
  if (rpi5_mem_fd < 0) return -errno;

  rpi5_rp1_bar =
      (volatile uint8_t *)mmap(NULL, RP1_BAR1_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, rpi5_mem_fd, RP1_BAR1);
  if (rpi5_rp1_bar == MAP_FAILED) {
    const int e = -errno;
    close(rpi5_mem_fd);
    rpi5_mem_fd = -1;
    return e;
  }

  rpi5_spi_base = rpi5_rp1_bar + RP1_SPI0_BASE;

  setup_spi0_pins();
  setup_spi0_ctrl(frequency_hz);

  return 1;
}

void rpi5_spi_xfer(uint8_t *rx, const uint8_t *tx, const size_t len) {
  if (!rpi5_spi_base) return;
  for (size_t i = 0; i < len; ++i) {
    // assert CS0 on each byte
    rpi5_wr32(rpi5_spi_base, DW_SPI_SER, 1u << 0);
    // write TX byte
    rpi5_wr32(rpi5_spi_base, DW_SPI_DR, tx[i]);
    while (rpi5_rd32(rpi5_spi_base, DW_SPI_RXFLR) == 0) {
      // wait for RX
    }
    // read RX byte
    rx[i] = (uint8_t)rpi5_rd32(rpi5_spi_base, DW_SPI_DR);
    // deassert CS0
    rpi5_wr32(rpi5_spi_base, DW_SPI_SER, 0);
  }
}
