#pragma once
#include <initializer_list>
#include <memory>

struct SpiDriver {
  virtual ~SpiDriver() = default;
  virtual int init(int frequency_hz) = 0;  // return 1 on match, 0 not-supported, <0 error
  virtual void xfer(uint8_t* rx, const uint8_t* tx, size_t len) = 0;

  static std::unique_ptr<SpiDriver> detect(int frequency_hz);

 protected:
  static uint32_t rd32(volatile void* base, uint32_t off);
  static void wr32(volatile void* base, uint32_t off, uint32_t value);
  static bool compat_contains_any(std::initializer_list<const char*> patterns);
};
