#pragma once

#include <cstdint>

#include "machine_definitions.h"

enum class SpiCommand : int32_t {
  Read = 0x72656164,   // "read" SPI command, LinuxCNC wants to read PRU state
  Write = 0x77726974,  // "writ" SPI command, LinuxCNC wants to write its state
};

static_assert(sizeof(SpiCommand) == sizeof(int32_t), "SpiCommand must be 32 bits");

constexpr auto kFixedPoint = 32;
constexpr long long kFixedOne = (1LL << kFixedPoint);
constexpr uint32_t kInitCounter = 0x696E6974;  // "init" initial value for PruState::packet_counter

#pragma pack(push, 4)

// struct for LinuxCNC -> PRU communication
struct LinuxCncState {
  SpiCommand command;
  int32_t output_vars[kNumOutputVars];
  uint16_t outputs;
  uint8_t stepgen_dir_mask;
  uint32_t steps_per_tick_cmd[kNumStepgens];  // Q0.32 steps per base tick (magnitude)
};

// struct for PRU -> LinuxCNC communication
struct PruState {
  // LPC1768's SPI TX FIFO is 16 frames, and DMA would pre-fill it at the end of the previous transfer,
  // so the first 16 bytes would be stale by the time the next transfer starts.
  // Let's put some junk there.
  uint8_t filler[16];
  uint32_t packet_counter;  // increments on each SpiCommand::Read; initialized to PRU_INIT_PKT
  int32_t input_vars[kNumInputVars];
  uint16_t inputs;
  int32_t stepgen_feedback[kNumStepgens];
};

#pragma pack(pop)

template <typename T>
class SpiBuffer {
  static constexpr size_t kLinuxCncStateSize = sizeof(LinuxCncState);
  static constexpr size_t kPruStateSize = sizeof(PruState);
  static constexpr size_t kBufSize = kPruStateSize > kLinuxCncStateSize ? kPruStateSize : kLinuxCncStateSize;

  static_assert(sizeof(T) <= kBufSize, "kBufSize too small for T");

 public:
  SpiBuffer() { new (bytes) T{}; }

  alignas(alignof(T)) uint8_t bytes[kBufSize]{};
  [[nodiscard]] size_t Size() const { return kBufSize; }
  T& AsStruct() { return *reinterpret_cast<T*>(bytes); }
};
