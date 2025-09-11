#pragma once

#include <bitset>
#include <concepts>
#include <limits>

template <std::floating_point DoubleType,
          DoubleType eps = std::numeric_limits<DoubleType>::epsilon()>
struct Double {
  using DoubleT = DoubleType;

  constexpr Double() = default;

  constexpr Double(const Double& /*unused*/) = default;

  constexpr Double(Double&& /*unused*/) = default;

  constexpr Double(const DoubleT& number) : number_(number) {}  // NOLINT

  template <typename ScalarType>
  requires std::is_constructible_v<DoubleT, ScalarType>
  constexpr Double(const ScalarType& scalar) : number_(scalar) {}  // NOLINT

  constexpr Double& operator=(const Double& /*unused*/) = default;

  constexpr Double& operator=(Double&& /*unused*/) = default;

  constexpr Double& operator=(const DoubleT& number) {
    number_ = number;
    return *this;
  }

  constexpr Double& operator+=(const Double& other) noexcept {
    number_ += other.number_;
    return *this;
  }

  [[nodiscard]] constexpr Double operator+(const Double& other) const noexcept {
    return Double(number_) += other.number_;
  }

  constexpr Double& operator-=(const Double& other) noexcept {
    number_ -= other.number_;
    return *this;
  }

  [[nodiscard]] constexpr Double operator-(const Double& other) const noexcept {
    return Double(number_) -= other.number_;
  }

  constexpr Double& operator*=(const Double& other) noexcept {
    number_ *= other.number_;
    return *this;
  }

  [[nodiscard]] constexpr Double operator*(const Double& other) const noexcept {
    return Double(number_) *= other.number_;
  }

  constexpr Double& operator/=(const Double& other) noexcept {
    number_ /= other.number_;
    return *this;
  }

  [[nodiscard]] constexpr Double operator/(const Double& other) const noexcept {
    return Double(number_) /= other.number_;
  }

  [[nodiscard]] constexpr Double operator-() const noexcept {
    return Double(-number_);
  }

  [[nodiscard]] constexpr Double operator+() const noexcept { return *this; }

  [[nodiscard]] friend constexpr bool operator==(const Double& lhs,
                                                 const Double& rhs) noexcept {
    using std::abs;
    return abs(lhs.number_ - rhs.number_) <= eps;
  }

  [[nodiscard]] friend constexpr auto operator<=>(const Double& lhs,
                                                  const Double& rhs) noexcept {
    if (lhs == rhs) {
      return std::strong_ordering::equal;
    }
    if (lhs.number_ < rhs.number_) {
      return std::strong_ordering::less;
    }
    return std::strong_ordering::greater;
  }

  friend std::istream& operator>>(std::istream& input, Double& dbl) {
    input >> dbl.number_;
    return input;
  }

  friend std::ostream& operator<<(std::ostream& output, const Double& dbl) {
    output << dbl.number_;
    return output;
  }

  [[nodiscard]] DoubleT constexpr AsBasicType() const noexcept {
    return number_;
  }

  constexpr operator DoubleT() const noexcept {  // NOLINT
    return AsBasicType();
  }

 private:
  DoubleT number_;
};

template <typename DoubleType, DoubleType eps>
struct std::hash<Double<DoubleType, eps>> {
  constexpr size_t operator()(const Double<DoubleType, eps>& dbl) const {
    return std::hash<DoubleType>{}(dbl);
  }
};

namespace std {

// NOLINTBEGIN

template <std::floating_point DoubleType, DoubleType eps>
class numeric_limits<Double<DoubleType, eps>> {
 public:
  using D = Double<DoubleType, eps>;
  using Base = numeric_limits<DoubleType>;

  static constexpr bool is_specialized = true;
  static constexpr bool is_signed = Base::is_signed;
  static constexpr bool is_integer = false;
  static constexpr bool is_exact = false;
  static constexpr bool has_infinity = Base::has_infinity;
  static constexpr bool has_quiet_NaN = Base::has_quiet_NaN;
  static constexpr bool has_signaling_NaN = Base::has_signaling_NaN;
  static constexpr float_denorm_style has_denorm = Base::has_denorm;
  static constexpr bool has_denorm_loss = Base::has_denorm_loss;
  static constexpr bool is_iec559 = Base::is_iec559;
  static constexpr bool is_bounded = Base::is_bounded;
  static constexpr bool is_modulo = false;
  static constexpr bool traps = Base::traps;
  static constexpr bool tinyness_before = Base::tinyness_before;
  static constexpr float_round_style round_style = Base::round_style;

  static constexpr int digits = Base::digits;
  static constexpr int digits10 = Base::digits10;
  static constexpr int max_digits10 = Base::max_digits10;
  static constexpr int radix = Base::radix;
  static constexpr int min_exponent = Base::min_exponent;
  static constexpr int min_exponent10 = Base::min_exponent10;
  static constexpr int max_exponent = Base::max_exponent;
  static constexpr int max_exponent10 = Base::max_exponent10;

  static constexpr D min() noexcept { return Base::min(); }
  static constexpr D max() noexcept { return Base::max(); }
  static constexpr D lowest() noexcept { return Base::lowest(); }
  static constexpr D epsilon() noexcept { return eps; }
  static constexpr D round_error() noexcept { return Base::round_error(); }
  static constexpr D infinity() noexcept { return Base::infinity(); }
  static constexpr D quiet_NaN() noexcept { return Base::quiet_NaN(); }
  static constexpr D signaling_NaN() noexcept { return Base::signaling_NaN(); }
  static constexpr D denorm_min() noexcept { return Base::denorm_min(); }
};
// NOLINTEND

}  // namespace std