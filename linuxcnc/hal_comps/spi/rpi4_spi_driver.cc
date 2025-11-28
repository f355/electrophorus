#include "rpi4_spi_driver.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <memory>

#include "rtapi.h"

/* Peripheral base and offsets (BCM2711) */
constexpr uint32_t BCM2711_PERI_BASE = 0xFE000000u;
constexpr uint32_t BCM_PERI_MAP_LEN = 0x00400000u;
constexpr uint32_t GPIO_OFFSET = 0x200000u;
constexpr uint32_t SPI0_OFFSET = 0x204000u;

/* SPI0 registers */
constexpr uint32_t SPI0_CS = 0x00;
constexpr uint32_t SPI0_FIFO = 0x04;
constexpr uint32_t SPI0_CLK = 0x08;

/* SPI0_CS bits */
constexpr uint32_t SPI0_CS_CS_MASK = 0x3u;
constexpr uint32_t SPI0_CS_CPHA = 1u << 2;
constexpr uint32_t SPI0_CS_CPOL = 1u << 3;
constexpr uint32_t SPI0_CS_CLEAR_RX = 1u << 4;
constexpr uint32_t SPI0_CS_CLEAR_TX = 1u << 5;
constexpr uint32_t SPI0_CS_TA = 1u << 7;
constexpr uint32_t SPI0_CS_DONE = 1u << 16;
constexpr uint32_t SPI0_CS_TXD = 1u << 18;

/* GPIO function select */
constexpr uint32_t GPFSEL0 = 0x00;
constexpr uint32_t GPFSEL1 = 0x04;
static uint32_t fsel_shift(const uint32_t p) { return 3u * (p % 10u); }
static uint32_t fsel_mask(const uint32_t p) { return 7u << fsel_shift(p); }
static uint32_t fsel_alt0(const uint32_t p) { return 4u << fsel_shift(p); }

Rpi4SpiDriver::~Rpi4SpiDriver() {
  if (bar_ && bar_ != MAP_FAILED) munmap(const_cast<uint8_t*>(bar_), BCM_PERI_MAP_LEN);
  if (mem_fd_ >= 0) close(mem_fd_);
}

int Rpi4SpiDriver::Init(int frequency_hz) {
  if (!CompatContainsAny({
          "raspberrypi,4",
          "raspberrypi,400",
          "raspberrypi,4-compute-module",
      }))
    return 0;  // not supported on this machine

  mem_fd_ = rtapi_open_as_root("/dev/mem", O_RDWR | O_SYNC);
  if (mem_fd_ < 0) return -errno;

  bar_ = static_cast<volatile uint8_t*>(
      mmap(nullptr, BCM_PERI_MAP_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd_, BCM2711_PERI_BASE));
  if (bar_ == MAP_FAILED) {
    const int e = -errno;
    close(mem_fd_);
    mem_fd_ = -1;
    return e;
  }

  gpio_ = bar_ + GPIO_OFFSET;
  spi0_ = bar_ + SPI0_OFFSET;

  // GPIO8..11 -> ALT0 for SPI0 (CE0, MISO, MOSI, SCLK)
  uint32_t v0 = Read(gpio_, GPFSEL0);
  v0 &= ~(fsel_mask(8) | fsel_mask(9));
  v0 |= (fsel_alt0(8) | fsel_alt0(9));
  Write(gpio_, GPFSEL0, v0);

  uint32_t v1 = Read(gpio_, GPFSEL1);
  v1 &= ~(fsel_mask(10) | fsel_mask(11));
  v1 |= (fsel_alt0(10) | fsel_alt0(11));
  Write(gpio_, GPFSEL1, v1);

  // SPI mode 1, 8-bit, clear FIFOs, set clock
  uint32_t cs = 0;
  cs |= SPI0_CS_CPHA;
  cs &= ~SPI0_CS_CPOL;
  cs &= ~SPI0_CS_CS_MASK;  // CS0
  cs |= (SPI0_CS_CLEAR_RX | SPI0_CS_CLEAR_TX);
  Write(spi0_, SPI0_CS, cs);

  if (frequency_hz <= 0) frequency_hz = 5'000'000;
  uint32_t div = 250000000u / static_cast<uint32_t>(frequency_hz);
  if (div & 1u) ++div;
  if (div < 2u) div = 2u;
  Write(spi0_, SPI0_CLK, div);
  return 1;
}

void Rpi4SpiDriver::Xfer(uint8_t* rx, const uint8_t* tx, const size_t len) {
  if (!spi0_) return;
  for (size_t i = 0; i < len; ++i) {
    uint32_t cs = Read(spi0_, SPI0_CS);
    cs |= (SPI0_CS_CLEAR_RX | SPI0_CS_CLEAR_TX);
    Write(spi0_, SPI0_CS, cs);

    cs = Read(spi0_, SPI0_CS);
    cs &= ~SPI0_CS_CS_MASK;  // CS0
    cs |= SPI0_CS_TA;        // assert CS
    Write(spi0_, SPI0_CS, cs);

    while ((Read(spi0_, SPI0_CS) & SPI0_CS_TXD) == 0) {
    }
    Write(spi0_, SPI0_FIFO, tx[i]);
    while ((Read(spi0_, SPI0_CS) & SPI0_CS_DONE) == 0) {
    }
    rx[i] = static_cast<uint8_t>(Read(spi0_, SPI0_FIFO));

    cs = Read(spi0_, SPI0_CS);
    cs &= ~SPI0_CS_TA;  // deassert CS
    Write(spi0_, SPI0_CS, cs);
  }
}
