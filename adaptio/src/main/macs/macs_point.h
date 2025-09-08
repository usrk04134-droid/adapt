#pragma once

#include <boost/uuid/uuid.hpp>
#include <cmath>
namespace macs {

// This is a coordinate in the machine coordinate system.
// This system has coordinates relative to a reference position
// in which the slide cross is centered. It's values are equal
// to the values reported by the PLC.

struct Point {
  double horizontal{};
  double vertical{};

  // Element-wise addition operator
  auto operator+(const Point& other) const -> Point {
    return {.horizontal = horizontal + other.horizontal, .vertical = vertical + other.vertical};
  };

  // Element-wise addition assignment operator
  auto operator+=(const Point& other) -> Point& {
    horizontal += other.horizontal;
    vertical   += other.vertical;

    return *this;
  };

  // Element-wise subtraction operator
  auto operator-(const Point& other) const -> Point {
    return {.horizontal = horizontal - other.horizontal, .vertical = vertical - other.vertical};
  };

  // Element-wise subtraction assignment operator
  auto operator-=(const Point& other) -> Point& {
    horizontal -= other.horizontal;
    vertical   -= other.vertical;

    return *this;
  };

  // Element-wise multiplication operator
  auto operator*(const Point& other) const -> Point {
    return {.horizontal = horizontal * other.horizontal, .vertical = vertical * other.vertical};
  };

  // Element-wise scalar multiplication operator
  auto operator*(int scalar) const -> Point {
    return {.horizontal = horizontal * scalar, .vertical = vertical * scalar};
  };

  // Element-wise multiplication assignment operator
  auto operator*=(const Point& other) -> Point& {
    horizontal *= other.horizontal;
    vertical   *= other.vertical;

    return *this;
  };

  // Element-wise scalar division operator
  auto operator/(int scalar) const -> Point {
    return {.horizontal = horizontal / scalar, .vertical = vertical / scalar};
  };

  // Element-wise division operator
  auto operator/(const Point& other) const -> Point {
    return {.horizontal = horizontal / other.horizontal, .vertical = vertical / other.vertical};
  };

  // Element-wise division assignment operator
  auto operator/=(const Point& other) -> Point& {
    horizontal /= other.horizontal;
    vertical   /= other.vertical;

    return *this;
  };

  // Norm (length) of vector from origin to this point
  auto Norm() const -> double { return std::sqrt((horizontal * horizontal) + (vertical * vertical)); };

  auto SquaredNorm() const -> double { return (horizontal * horizontal) + (vertical * vertical); };
};

}  // namespace macs
