/*
 BCM2711 (Raspberry Pi 4) SPI0 + GPIO minimal access for SPI0 CS0, mode 1, 8-bit

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
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

static volatile uint8_t *rpi4_bcm_bar = 0;
static volatile uint8_t *rpi4_gpio_base = 0;
static volatile uint8_t *rpi4_spi0_base = 0;
static int rpi4_mem_fd = -1;

/* Peripheral base and offsets (BCM2711) */
#define BCM2711_PERI_BASE 0xFE000000u
#define BCM_PERI_MAP_LEN 0x00400000u /* 4 MiB covers GPIO (0x200000) and SPI0 (0x204000) */
#define GPIO_OFFSET 0x200000u
#define SPI0_OFFSET 0x204000u

/* GPIO registers */
#define GPFSEL0 0x00
#define GPFSEL1 0x04

/* SPI0 registers */
#define SPI0_CS 0x00
#define SPI0_FIFO 0x04
#define SPI0_CLK 0x08

/* SPI0_CS bits */
#define SPI0_CS_CS_MASK 0x3u   /* bits 1:0 */
#define SPI0_CS_CPHA (1u << 2) /* mode bit */
#define SPI0_CS_CPOL (1u << 3)
#define SPI0_CS_CLEAR_RX (1u << 4)
#define SPI0_CS_CLEAR_TX (1u << 5)
#define SPI0_CS_TA (1u << 7)
#define SPI0_CS_DONE (1u << 16)
#define SPI0_CS_RXD (1u << 17)
#define SPI0_CS_TXD (1u << 18)

/* GPIO function select helpers */
#define FSEL_SHIFT(pin) (3u * ((pin) % 10u))
#define FSEL_MASK(pin) (7u << FSEL_SHIFT(pin))
#define FSEL_ALT0(pin) (4u << FSEL_SHIFT(pin))

static inline uint32_t rpi4_rd32(volatile uint8_t *base, uint32_t off) { return *(volatile uint32_t *)(base + off); }
static inline void rpi4_wr32(volatile uint8_t *base, uint32_t off, uint32_t v) {
  *(volatile uint32_t *)(base + off) = v;
}

static bool is_rpi4(void) {
  int fd = open("/proc/device-tree/compatible", O_RDONLY);
  if (fd < 0) return false;
  char buf[256];
  ssize_t n = read(fd, buf, sizeof(buf));
  close(fd);
  if (n <= 0) return false;
  /* Look for common Pi4 compatibles */
  if (memmem(buf, (size_t)n, "raspberrypi,4", 14)) return true;
  if (memmem(buf, (size_t)n, "raspberrypi,400", 16)) return true;
  if (memmem(buf, (size_t)n, "raspberrypi,4-compute-module", 29)) return true;
  return false;
}

static void gpio_set_alt0_spi0(void) {
  /* Set GPIO 8,9,10,11 to ALT0 (SPI0 CE0/MISO/MOSI/SCLK) */
  /* GPFSEL0: pins 0..9, GPFSEL1: 10..19; 3 bits per pin, ALT0 = 0b100 */
  uint32_t v0 = rpi4_rd32(rpi4_gpio_base, GPFSEL0);
  v0 &= ~(FSEL_MASK(8) | FSEL_MASK(9)); /* clear 8,9 */
  v0 |= (FSEL_ALT0(8) | FSEL_ALT0(9));  /* ALT0 */
  rpi4_wr32(rpi4_gpio_base, GPFSEL0, v0);

  uint32_t v1 = rpi4_rd32(rpi4_gpio_base, GPFSEL1);
  v1 &= ~(FSEL_MASK(10) | FSEL_MASK(11)); /* clear 10,11 */
  v1 |= (FSEL_ALT0(10) | FSEL_ALT0(11));  /* ALT0 */
  rpi4_wr32(rpi4_gpio_base, GPFSEL1, v1);
}

static void spi0_setup_mode1_8bit(int frequency_hz) {
  /* Clear FIFOs, set mode1 (CPHA=1, CPOL=0), CS=0 */
  uint32_t cs = 0;
  cs |= SPI0_CS_CPHA; /* mode 1 */
  cs &= ~SPI0_CS_CPOL;
  cs &= ~SPI0_CS_CS_MASK; /* CS0 */
  cs |= (SPI0_CS_CLEAR_RX | SPI0_CS_CLEAR_TX);
  rpi4_wr32(rpi4_spi0_base, SPI0_CS, cs);

  /* Clock divider: core clock ~250 MHz; divider must be even, >= 2 */
  if (frequency_hz <= 0) frequency_hz = 5000000;
  uint32_t cdiv = 250000000u / (uint32_t)frequency_hz;
  if ((cdiv & 1u) != 0) cdiv++; /* even */
  if (cdiv < 2u) cdiv = 2u;
  rpi4_wr32(rpi4_spi0_base, SPI0_CLK, cdiv);
}

int rpi4_spi_init(int frequency_hz) {
  if (!is_rpi4()) return 0;

  rpi4_mem_fd = rtapi_open_as_root("/dev/mem", O_RDWR | O_SYNC);
  if (rpi4_mem_fd < 0) return -errno;

  rpi4_bcm_bar = (volatile uint8_t *)mmap(NULL, BCM_PERI_MAP_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, rpi4_mem_fd,
                                          (off_t)BCM2711_PERI_BASE);
  if (rpi4_bcm_bar == MAP_FAILED) {
    int e = -errno;
    close(rpi4_mem_fd);
    rpi4_mem_fd = -1;
    return e;
  }

  rpi4_gpio_base = rpi4_bcm_bar + GPIO_OFFSET;
  rpi4_spi0_base = rpi4_bcm_bar + SPI0_OFFSET;

  gpio_set_alt0_spi0();
  spi0_setup_mode1_8bit(frequency_hz);
  return 1;
}

void rpi4_spi_xfer(uint8_t *rx, const uint8_t *tx, size_t len) {
  if (!rpi4_spi0_base) return;
  for (size_t i = 0; i < len; ++i) {
    /* Clear TX/RX FIFOs and start transfer (TA) */
    uint32_t cs = rpi4_rd32(rpi4_spi0_base, SPI0_CS);
    cs |= (SPI0_CS_CLEAR_RX | SPI0_CS_CLEAR_TX);
    rpi4_wr32(rpi4_spi0_base, SPI0_CS, cs);

    cs = rpi4_rd32(rpi4_spi0_base, SPI0_CS);
    cs &= ~SPI0_CS_CS_MASK; /* CS0 */
    cs |= SPI0_CS_TA;       /* transfer active */
    rpi4_wr32(rpi4_spi0_base, SPI0_CS, cs);

    /* Wait for TXD, then write one byte */
    while ((rpi4_rd32(rpi4_spi0_base, SPI0_CS) & SPI0_CS_TXD) == 0) {
    }
    rpi4_wr32(rpi4_spi0_base, SPI0_FIFO, (uint32_t)tx[i]);

    /* Wait for DONE */
    while ((rpi4_rd32(rpi4_spi0_base, SPI0_CS) & SPI0_CS_DONE) == 0) {
    }

    /* Read back */
    uint8_t rb = (uint8_t)rpi4_rd32(rpi4_spi0_base, SPI0_FIFO);
    rx[i] = rb;

    /* End transfer (deassert CS) */
    cs = rpi4_rd32(rpi4_spi0_base, SPI0_CS);
    cs &= ~SPI0_CS_TA;
    rpi4_wr32(rpi4_spi0_base, SPI0_CS, cs);
  }
}
