#ifndef FIXED32_32_H
#define FIXED32_32_H

#include <cstdint>
#include <type_traits>

/**
 * Fixed32_32 - A 32.32 fixed-point number implementation
 *
 * Uses 64 bits total: 32 bits for integer part, 32 bits for fractional part
 * Supports arithmetic operations, conversions, and volatile qualifier usage
 */
class Fixed32_32 {
  static constexpr int64_t FRACTIONAL_BITS = 32;
  static constexpr int64_t SCALE = 1LL << FRACTIONAL_BITS;

  int64_t value_;

 public:
  // Default constructor
  constexpr Fixed32_32() noexcept : value_(0) {}

  // Copy constructor
  constexpr Fixed32_32(const Fixed32_32& other) noexcept = default;

  // Copy constructor for volatile
  Fixed32_32(const volatile Fixed32_32& other) noexcept {
    const int64_t other_val = other.value_;
    value_ = other_val;
  }

  // Constructor from raw value (private use)
  explicit constexpr Fixed32_32(int64_t raw_value, bool) noexcept : value_(raw_value) {}

  // Implicit conversion from integer types
  template <typename T>
  constexpr Fixed32_32(T val, std::enable_if_t<std::is_integral_v<T>>* = nullptr) noexcept
      : value_(static_cast<int64_t>(val) << FRACTIONAL_BITS) {}

  // Assignment operators
  Fixed32_32& operator=(const Fixed32_32& other) noexcept {
    value_ = other.value_;
    return *this;
  }

  // Helper method for volatile assignment to avoid compiler warnings
  void assign_from(const Fixed32_32& other) volatile noexcept { value_ = other.value_; }

  Fixed32_32& operator=(const volatile Fixed32_32& other) {
    const int64_t other_val = other.value_;
    value_ = other_val;
    return *this;
  }

  volatile Fixed32_32& operator=(const volatile Fixed32_32& other) volatile {
    const int64_t other_val = other.value_;
    value_ = other_val;
    return *this;
  }

  // Template assignment from other types (excluding Fixed32_32 to avoid conflicts)
  template <typename T>
  std::enable_if_t<!std::is_same_v<std::decay_t<T>, Fixed32_32>, Fixed32_32&> operator=(T val) {
    if constexpr (std::is_integral_v<T>) {
      value_ = static_cast<int64_t>(val) << FRACTIONAL_BITS;
    } else {
      value_ = Fixed32_32(val).value_;
    }
    return *this;
  }

  template <typename T>
  std::enable_if_t<!std::is_same_v<std::decay_t<T>, Fixed32_32>, volatile Fixed32_32&> operator=(T val) volatile {
    if constexpr (std::is_integral_v<T>) {
      value_ = static_cast<int64_t>(val) << FRACTIONAL_BITS;
    } else {
      value_ = Fixed32_32(val).value_;
    }
    return *this;
  }

  // Conversion to integer (truncates fractional part)
  explicit constexpr operator int32_t() const { return static_cast<int32_t>(value_ >> FRACTIONAL_BITS); }

  explicit operator int32_t() const volatile {
    const int64_t this_val = value_;
    return static_cast<int32_t>(this_val >> FRACTIONAL_BITS);
  }

  // Arithmetic operators
  constexpr Fixed32_32 operator+(const Fixed32_32& other) const { return Fixed32_32(value_ + other.value_, true); }

  Fixed32_32 operator+(const volatile Fixed32_32& other) const {
    const int64_t other_val = other.value_;
    return Fixed32_32(value_ + other_val, true);
  }

  Fixed32_32 operator+(const Fixed32_32& other) const volatile {
    const int64_t this_val = value_;
    return Fixed32_32(this_val + other.value_, true);
  }

  Fixed32_32 operator+(const volatile Fixed32_32& other) const volatile {
    const int64_t this_val = value_;
    const int64_t other_val = other.value_;
    return Fixed32_32(this_val + other_val, true);
  }

  constexpr Fixed32_32 operator-(const Fixed32_32& other) const { return Fixed32_32(value_ - other.value_, true); }

  Fixed32_32 operator-(const volatile Fixed32_32& other) const {
    const int64_t other_val = other.value_;
    return Fixed32_32(value_ - other_val, true);
  }

  Fixed32_32 operator-(const Fixed32_32& other) const volatile {
    const int64_t this_val = value_;
    return Fixed32_32(this_val - other.value_, true);
  }

  Fixed32_32 operator-(const volatile Fixed32_32& other) const volatile {
    const int64_t this_val = value_;
    const int64_t other_val = other.value_;
    return Fixed32_32(this_val - other_val, true);
  }

  // Unary negation
  constexpr Fixed32_32 operator-() const { return Fixed32_32(-value_, true); }

  Fixed32_32 operator-() const volatile {
    const int64_t this_val = value_;
    return Fixed32_32(-this_val, true);
  }

 private:
  // Helper function for 64-bit multiplication with 64-bit result (checking for overflow)
  static constexpr int64_t multiply_fixed(const int64_t a, const int64_t b) {
    // For fixed-point multiplication: (a * b) >> FRACTIONAL_BITS
    // We need to compute the high 64 bits of a * b, then combine with low bits

    const auto a_abs = static_cast<uint64_t>(a < 0 ? -a : a);
    const auto b_abs = static_cast<uint64_t>(b < 0 ? -b : b);

    const auto a_lo = static_cast<uint32_t>(a_abs);
    const auto a_hi = static_cast<uint32_t>(a_abs >> 32);
    const auto b_lo = static_cast<uint32_t>(b_abs);
    const auto b_hi = static_cast<uint32_t>(b_abs >> 32);

    const uint64_t p0 = static_cast<uint64_t>(a_lo) * b_lo;
    const uint64_t p1 = static_cast<uint64_t>(a_lo) * b_hi;
    const uint64_t p2 = static_cast<uint64_t>(a_hi) * b_lo;
    const uint64_t p3 = static_cast<uint64_t>(a_hi) * b_hi;

    const uint64_t low = p0 + ((p1 & 0xFFFFFFFF) << 32) + ((p2 & 0xFFFFFFFF) << 32);
    const uint64_t carry = ((p0 >> 32) + (p1 & 0xFFFFFFFF) + (p2 & 0xFFFFFFFF)) >> 32;
    const uint64_t high = p3 + (p1 >> 32) + (p2 >> 32) + carry;

    // Shift right by FRACTIONAL_BITS (32)
    const uint64_t result = (high << (64 - FRACTIONAL_BITS)) | (low >> FRACTIONAL_BITS);

    return ((a < 0) ^ (b < 0)) ? -static_cast<int64_t>(result) : static_cast<int64_t>(result);
  }

  // Helper function for fixed-point division using integer arithmetic only
  static constexpr int64_t divide_fixed(const int64_t dividend, const int64_t divisor) {
    if (divisor == 0) {
      return dividend >= 0 ? INT64_MAX : INT64_MIN;
    }

    const bool negative = (dividend < 0) ^ (divisor < 0);

    // Work with absolute values
    const auto abs_dividend = static_cast<uint64_t>(dividend < 0 ? -dividend : dividend);
    const auto abs_divisor = static_cast<uint64_t>(divisor < 0 ? -divisor : divisor);

    // For fixed-point division, we need to compute (dividend << FRACTIONAL_BITS) / divisor
    // To avoid overflow, we'll use long division algorithm

    uint64_t quotient = 0;
    uint64_t remainder = 0;

    // Process 64 + 32 = 96 bits (original 64 bits + 32 fractional bits)
    for (int i = 63; i >= -32; --i) {
      remainder <<= 1;

      if (i >= 0 && (abs_dividend & 1ULL << i)) remainder |= 1;

      // For i < 0, we're processing the "shifted" fractional bits (all zeros)
      if (remainder >= abs_divisor) {
        remainder -= abs_divisor;
        quotient |= (1ULL << (i + 32));
      }
    }

    // Clamp to valid range
    if (quotient > static_cast<uint64_t>(INT64_MAX)) {
      quotient = static_cast<uint64_t>(INT64_MAX);
    }

    return negative ? -static_cast<int64_t>(quotient) : static_cast<int64_t>(quotient);
  }

 public:
  constexpr Fixed32_32 operator*(const Fixed32_32& other) const {
    return Fixed32_32(multiply_fixed(value_, other.value_), true);
  }

  Fixed32_32 operator*(const volatile Fixed32_32& other) const {
    const int64_t other_val = other.value_;
    return Fixed32_32(multiply_fixed(value_, other_val), true);
  }

  Fixed32_32 operator*(const Fixed32_32& other) const volatile {
    const int64_t this_val = value_;
    return Fixed32_32(multiply_fixed(this_val, other.value_), true);
  }

  Fixed32_32 operator*(const volatile Fixed32_32& other) const volatile {
    const int64_t this_val = value_;
    const int64_t other_val = other.value_;
    return Fixed32_32(multiply_fixed(this_val, other_val), true);
  }

  constexpr Fixed32_32 operator/(const Fixed32_32& other) const {
    return Fixed32_32(divide_fixed(value_, other.value_), true);
  }

  Fixed32_32 operator/(const volatile Fixed32_32& other) const {
    const int64_t other_val = other.value_;
    return Fixed32_32(divide_fixed(value_, other_val), true);
  }

  Fixed32_32 operator/(const Fixed32_32& other) const volatile {
    const int64_t this_val = value_;
    return Fixed32_32(divide_fixed(this_val, other.value_), true);
  }

  Fixed32_32 operator/(const volatile Fixed32_32& other) const volatile {
    const int64_t this_val = value_;
    const int64_t other_val = other.value_;
    return Fixed32_32(divide_fixed(this_val, other_val), true);
  }

  // Compound assignment operators
  Fixed32_32& operator+=(const Fixed32_32& other) {
    value_ += other.value_;
    return *this;
  }

  volatile Fixed32_32& operator+=(const Fixed32_32& other) volatile {
    value_ += other.value_;
    return *this;
  }

  Fixed32_32& operator-=(const Fixed32_32& other) {
    value_ -= other.value_;
    return *this;
  }

  volatile Fixed32_32& operator-=(const Fixed32_32& other) volatile {
    value_ -= other.value_;
    return *this;
  }

  Fixed32_32& operator*=(const Fixed32_32& other) {
    value_ = multiply_fixed(value_, other.value_);
    return *this;
  }

  volatile Fixed32_32& operator*=(const Fixed32_32& other) volatile {
    const int64_t this_val = value_;
    value_ = multiply_fixed(this_val, other.value_);
    return *this;
  }

  Fixed32_32& operator/=(const Fixed32_32& other) {
    value_ = divide_fixed(value_, other.value_);
    return *this;
  }

  volatile Fixed32_32& operator/=(const Fixed32_32& other) volatile {
    const int64_t this_val = value_;
    value_ = divide_fixed(this_val, other.value_);
    return *this;
  }

  // Comparison operators
  constexpr bool operator==(const Fixed32_32& other) const { return value_ == other.value_; }

  bool operator==(const volatile Fixed32_32& other) const {
    const int64_t other_val = other.value_;
    return value_ == other_val;
  }

  bool operator==(const Fixed32_32& other) const volatile {
    const int64_t this_val = value_;
    return this_val == other.value_;
  }

  bool operator==(const volatile Fixed32_32& other) const volatile {
    const int64_t this_val = value_;
    const int64_t other_val = other.value_;
    return this_val == other_val;
  }

  constexpr bool operator!=(const Fixed32_32& other) const { return value_ != other.value_; }

  bool operator!=(const volatile Fixed32_32& other) const {
    const int64_t other_val = other.value_;
    return value_ != other_val;
  }

  bool operator!=(const Fixed32_32& other) const volatile {
    const int64_t this_val = value_;
    return this_val != other.value_;
  }

  bool operator!=(const volatile Fixed32_32& other) const volatile {
    const int64_t this_val = value_;
    const int64_t other_val = other.value_;
    return this_val != other_val;
  }

  constexpr bool operator<(const Fixed32_32& other) const { return value_ < other.value_; }

  bool operator<(const volatile Fixed32_32& other) const {
    const int64_t other_val = other.value_;
    return value_ < other_val;
  }

  bool operator<(const Fixed32_32& other) const volatile {
    const int64_t this_val = value_;
    return this_val < other.value_;
  }

  bool operator<(const volatile Fixed32_32& other) const volatile {
    const int64_t this_val = value_;
    const int64_t other_val = other.value_;
    return this_val < other_val;
  }

  constexpr bool operator<=(const Fixed32_32& other) const { return value_ <= other.value_; }

  bool operator<=(const volatile Fixed32_32& other) const {
    const int64_t other_val = other.value_;
    return value_ <= other_val;
  }

  bool operator<=(const Fixed32_32& other) const volatile {
    const int64_t this_val = value_;
    return this_val <= other.value_;
  }

  bool operator<=(const volatile Fixed32_32& other) const volatile {
    const int64_t this_val = value_;
    const int64_t other_val = other.value_;
    return this_val <= other_val;
  }

  constexpr bool operator>(const Fixed32_32& other) const { return value_ > other.value_; }

  bool operator>(const volatile Fixed32_32& other) const {
    const int64_t other_val = other.value_;
    return value_ > other_val;
  }

  bool operator>(const Fixed32_32& other) const volatile {
    const int64_t this_val = value_;
    return this_val > other.value_;
  }

  bool operator>(const volatile Fixed32_32& other) const volatile {
    const int64_t this_val = value_;
    const int64_t other_val = other.value_;
    return this_val > other_val;
  }

  constexpr bool operator>=(const Fixed32_32& other) const { return value_ >= other.value_; }

  bool operator>=(const volatile Fixed32_32& other) const {
    const int64_t other_val = other.value_;
    return value_ >= other_val;
  }

  bool operator>=(const Fixed32_32& other) const volatile {
    const int64_t this_val = value_;
    return this_val >= other.value_;
  }

  bool operator>=(const volatile Fixed32_32& other) const volatile {
    const int64_t this_val = value_;
    const int64_t other_val = other.value_;
    return this_val >= other_val;
  }

  // Absolute value
  [[nodiscard]] constexpr Fixed32_32 abs() const { return Fixed32_32(value_ < 0 ? -value_ : value_, true); }

  [[nodiscard]] Fixed32_32 abs() const volatile {
    const int64_t this_val = value_;
    return Fixed32_32(this_val < 0 ? -this_val : this_val, true);
  }

  // Custom function as requested
  [[nodiscard]] constexpr bool one_over(const Fixed32_32& other) const { return (this->value_ ^ other.value_) & SCALE; }

  // Access to raw value (for debugging/serialization)
  [[nodiscard]] constexpr int64_t raw_value() const { return value_; }
  [[nodiscard]] int64_t raw_value() const volatile {
    const int64_t this_val = value_;
    return this_val;
  }

  // Static factory method for creating from raw value
  static constexpr Fixed32_32 from_raw(const int64_t raw) { return Fixed32_32(raw, true); }
};

#endif  // FIXED32_32_H
