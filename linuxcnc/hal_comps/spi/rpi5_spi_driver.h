#pragma once
#include <memory>

#include "spi_driver.h"

class Rpi5SpiDriver final : public SpiDriver {
 public:
  ~Rpi5SpiDriver() override;
  int Init(int frequency_hz) override;
  void Xfer(uint8_t* rx, const uint8_t* tx, size_t len) override;

 private:
  volatile uint8_t* bar_ = nullptr;
  volatile uint8_t* spi_ = nullptr;
  int mem_fd_ = -1;
};
