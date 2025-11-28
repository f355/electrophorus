#pragma once
#include <memory>

#include "spi_driver.h"

class Rpi4SpiDriver final : public SpiDriver {
 public:
  ~Rpi4SpiDriver() override;
  int Init(int frequency_hz) override;
  void Xfer(uint8_t* rx, const uint8_t* tx, size_t len) override;

 private:
  volatile uint8_t* bar_ = nullptr;
  volatile uint8_t* gpio_ = nullptr;
  volatile uint8_t* spi0_ = nullptr;
  int mem_fd_ = -1;
};
