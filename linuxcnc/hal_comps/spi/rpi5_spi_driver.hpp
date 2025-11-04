#pragma once
#include <memory>

#include "spi_driver.hpp"

class Rpi5SpiDriver final : public SpiDriver {
 public:
  int init(int frequency_hz) override;
  void xfer(uint8_t* rx, const uint8_t* tx, size_t len) override;

 private:
  volatile uint8_t* bar = nullptr;
  volatile uint8_t* spi = nullptr;
  int mem_fd = -1;
};
