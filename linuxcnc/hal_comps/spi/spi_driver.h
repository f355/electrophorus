#pragma once

#include <initializer_list>
#include <memory>

struct SpiDriver {
  virtual ~SpiDriver() = default;
  virtual int Init(int frequency_hz) = 0;  // return 1 on match, 0 not-supported, <0 error
  virtual void Xfer(uint8_t* rx, const uint8_t* tx, size_t len) = 0;

  static std::unique_ptr<SpiDriver> Detect(int frequency_hz);

 protected:
  static uint32_t Read(volatile void* base, uint32_t off);
  static void Write(volatile void* base, uint32_t off, uint32_t value);
  static bool CompatContainsAny(std::initializer_list<const char*> patterns);
};
