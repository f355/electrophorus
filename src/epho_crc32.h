#ifndef EPHO_CRC32_H
#define EPHO_CRC32_H

#include <array>
#include <utility>

// Header-only CRC32 (IEEE 802.3, reflected) with constexpr 256-entry table
// Designed for ISR streaming: init/update/finalize only; no dynamic state.
namespace epho_crc32_detail {
static constexpr uint32_t POLY = 0xEDB88320u;

constexpr uint32_t entry(uint32_t c) {
  for (int k = 0; k < 8; ++k) c = (c & 1u) ? (c >> 1) ^ POLY : (c >> 1);
  return c;
}

template <size_t... I>
constexpr std::array<uint32_t, 256> make_impl(std::index_sequence<I...>) {
  return {entry(I)...};
}

constexpr std::array<uint32_t, 256> make() { return make_impl(std::make_index_sequence<256>{}); }

// Internal linkage to avoid ODR issues when included in multiple TUs
static constexpr auto TABLE = make();
}  // namespace epho_crc32_detail

struct EphoCRC32 {
  static void init(uint32_t &crc) { crc = 0xFFFFFFFFu; }
  static void update(uint32_t &crc, const uint8_t byte) {
    crc = epho_crc32_detail::TABLE[(crc ^ byte) & 0xFFu] ^ (crc >> 8);
  }
  static uint32_t finalize(const uint32_t crc) { return ~crc; }
};

#endif  // EPHO_CRC32_H
