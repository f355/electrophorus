#include "spi_driver.h"

#include <fcntl.h>
#include <unistd.h>

#include <cstring>
#include <initializer_list>
#include <memory>

#include "rpi4_spi_driver.h"
#include "rpi5_spi_driver.h"

uint32_t SpiDriver::Read(volatile void* base, const uint32_t off) {
  return *reinterpret_cast<volatile uint32_t*>(static_cast<volatile uint8_t*>(base) + off);
}
void SpiDriver::Write(volatile void* base, const uint32_t off, const uint32_t value) {
  *reinterpret_cast<volatile uint32_t*>(static_cast<volatile uint8_t*>(base) + off) = value;
}

bool SpiDriver::CompatContainsAny(const std::initializer_list<const char*> patterns) {
  const int fd = open("/proc/device-tree/compatible", O_RDONLY);
  if (fd < 0) return false;
  char buf[1024];
  const ssize_t n = read(fd, buf, sizeof(buf));
  close(fd);
  if (n <= 0) return false;
  for (const char* p : patterns) {
    if (p && memmem(buf, static_cast<size_t>(n), p, strlen(p))) return true;
  }
  return false;
}

std::unique_ptr<SpiDriver> SpiDriver::Detect(const int frequency_hz) {
  if (auto d = std::make_unique<Rpi5SpiDriver>(); d->Init(frequency_hz) > 0) return d;
  if (auto d = std::make_unique<Rpi4SpiDriver>(); d->Init(frequency_hz) > 0) return d;
  return nullptr;
}
