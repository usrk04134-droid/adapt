#pragma once

#include <cassert>
#include <cmath>
#include <numbers>

namespace common::math {

inline auto DegToRad(double val) -> double { return val * std::numbers::pi / 180.; }
inline auto RadToDeg(double val) -> double { return val * 180. / std::numbers::pi; }
inline auto CmMinToMmSec(double val) -> double { return val / 6.; }
inline auto MmSecToCmMin(double val) -> double { return val * 6.; }

inline auto LinearToAngular(double val, double radius) {
  /* works for both distance and velocity */
  assert(radius > 0.0);
  return val / radius;
}

inline auto AngularToLinear(double val, double radius) {
  /* works for both distance and velocity */
  return val * radius;
}

inline auto WrappedDist(double p1, double p2, double wrap_value) {
  auto wrapped_diff = std::fmod(p2 - p1, wrap_value);
  if (wrapped_diff > wrap_value / 2.0) {
    wrapped_diff -= wrap_value;
  } else if (wrapped_diff <= -wrap_value / 2.0) {
    wrapped_diff += wrap_value;
  }
  return wrapped_diff;
}

inline auto WrappedDistAbs(double p1, double p2, double wrap_value) {
  auto const dist = common::math::WrappedDist(p1, p2, wrap_value);
  return dist < 0.0 ? dist + wrap_value : dist;
}

}  // namespace common::math
