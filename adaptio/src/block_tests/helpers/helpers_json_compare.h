#pragma once

#include <cmath>
#include <nlohmann/json.hpp>

inline const double JSON_FLOAT_TOLERANCE = 1e-5;

// NOLINTNEXTLINE(misc-no-recursion)
inline auto JsonEqualWithTolerance(nlohmann::json const& a, nlohmann::json const& b,
                                   double epsilon = JSON_FLOAT_TOLERANCE) -> bool {
  // Handle number comparison across types (float vs int)
  if (a.type() == nlohmann::json::value_t::number_float || b.type() == nlohmann::json::value_t::number_float ||
      (a.is_number() && b.is_number())) {
    return std::fabs(a.get<double>() - b.get<double>()) <= epsilon;
  }

  // Types must match for other comparisons
  if (a.type() != b.type()) {
    return false;
  }

  switch (a.type()) {
    case nlohmann::json::value_t::object:
      if (a.size() != b.size()) {
        return false;
      }
      for (auto it = a.begin(); it != a.end(); ++it) {
        if (!b.contains(it.key()) || !JsonEqualWithTolerance(it.value(), b.at(it.key()), epsilon)) {
          return false;
        }
      }
      return true;

    case nlohmann::json::value_t::array:
      if (a.size() != b.size()) {
        return false;
      }
      for (size_t i = 0; i < a.size(); ++i) {
        if (!JsonEqualWithTolerance(a[i], b[i], epsilon)) {
          return false;
        }
      }
      return true;

    default:
      return a == b;
  }
}
