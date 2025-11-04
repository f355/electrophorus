#pragma once

#include <cstddef>

#include "machine_definitions.h"

using namespace std;

enum class SpiCommand : int32_t {
  Read = 0x72656164,   // "read" SPI command, LinuxCNC wants to read PRU state
  Write = 0x77726974,  // "writ" SPI command, LinuxCNC wants to write its state
};

static_assert(sizeof(SpiCommand) == sizeof(int32_t), "SpiCommand must be 32 bits");

constexpr auto FIXED_POINT = 32;
constexpr long long FIXED_ONE = (1LL << FIXED_POINT);
constexpr uint32_t PRU_INIT_PKT = 0x696E6974;  // "init" initial value for PruState::packet_counter

#pragma pack(push, 4)

// struct for LinuxCNC -> PRU communication
struct LinuxCncState {
  SpiCommand command;
  int32_t output_vars[OUTPUT_VARS];
  uint16_t outputs;
  uint8_t stepgen_dir_mask;
  uint32_t steps_per_tick_cmd[STEPGENS];  // Q0.32 steps per base tick (magnitude)
};

// struct for PRU -> LinuxCNC communication
struct PruState {
  // LPC1768's SPI TX FIFO is 16 frames, and DMA would pre-fill it at the end of the previous transfer,
  // so the first 16 bytes would be stale by the time the next transfer starts.
  // Let's put some junk there. f
  uint8_t filler[16];
  uint32_t packet_counter;  // increments on each SpiCommand::Read; initialized to PRU_INIT_PKT
  int32_t input_vars[INPUT_VARS];
  uint16_t inputs;
  int32_t stepgen_feedback[STEPGENS];
};

#pragma pack(pop)

template <typename T>
class SpiBuffer {
  static constexpr size_t tx_size = sizeof(LinuxCncState);
  static constexpr size_t rx_size = sizeof(PruState);
  static constexpr size_t buf_size = rx_size > tx_size ? rx_size : tx_size;

  static_assert(sizeof(T) <= buf_size, "buf_size too small for T");

 public:
  static constexpr size_t buffer_size() { return buf_size; }

  SpiBuffer() { new (bytes) T{}; }

  alignas(alignof(T)) uint8_t bytes[buf_size]{};
  [[nodiscard]] size_t size() const { return buf_size; }
  T& state() { return *reinterpret_cast<T*>(bytes); }
};
