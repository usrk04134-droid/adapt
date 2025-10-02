#pragma once

#include <boost/uuid/uuid.hpp>
#include <cmath>

namespace common::groove {

struct Point {
  double horizontal{};
  double vertical{};

  auto operator+(const Point& other) const -> Point {
    return {.horizontal = horizontal + other.horizontal, .vertical = vertical + other.vertical};
  };

  auto operator+=(const Point& other) -> Point& {
    horizontal += other.horizontal;
    vertical   += other.vertical;

    return *this;
  };

  auto operator-(const Point& other) const -> Point {
    return {.horizontal = horizontal - other.horizontal, .vertical = vertical - other.vertical};
  };

  auto operator-=(const Point& other) -> Point& {
    horizontal -= other.horizontal;
    vertical   -= other.vertical;

    return *this;
  };

  auto operator*(const Point& other) const -> Point {
    return {.horizontal = horizontal * other.horizontal, .vertical = vertical * other.vertical};
  };

  auto operator*(int scalar) const -> Point {
    return {.horizontal = horizontal * scalar, .vertical = vertical * scalar};
  };

  auto operator*=(const Point& other) -> Point& {
    horizontal *= other.horizontal;
    vertical   *= other.vertical;

    return *this;
  };

  auto operator/(int scalar) const -> Point {
    return {.horizontal = horizontal / scalar, .vertical = vertical / scalar};
  };

  auto operator/(const Point& other) const -> Point {
    return {.horizontal = horizontal / other.horizontal, .vertical = vertical / other.vertical};
  };

  auto operator/=(const Point& other) -> Point& {
    horizontal /= other.horizontal;
    vertical   /= other.vertical;

    return *this;
  };

  auto Norm() const -> double { return std::sqrt((horizontal * horizontal) + (vertical * vertical)); };

  auto SquaredNorm() const -> double { return (horizontal * horizontal) + (vertical * vertical); };
};

}  // namespace common::groove

