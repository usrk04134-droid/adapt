#include "core/math/lin_interp.h"

#include <assert.h>

#include <cstddef>
#include <tuple>
#include <vector>

auto get_slope(const std::tuple<double, double> &p1, const std::tuple<double, double> &p2) -> double {
  return (std::get<1>(p2) - std::get<1>(p1)) / (std::get<0>(p2) - std::get<0>(p1));
}

auto calc_interp_2d(double x1, double y1, double m, double x) { return y1 + m * (x - x1); }

auto get_segment(double x_value, const std::vector<std::tuple<double, double>> &segments) -> unsigned long {
  assert(segments.size() >= 2);

  for (unsigned long segment = 0; segment < segments.size() - 1; segment++) {
    if (x_value <= std::get<0>(segments[segment + 1])) {
      return segment;
    }
  }
  // If x value is larger than last segment point x value, use the last segment, i.e. return second last point.
  return segments.size() - 2;
}

auto core::math::lin_interp::lin_interp_2d(const std::vector<double> &x_values,
                                           const std::vector<std::tuple<double, double>> &segments)
    -> std::vector<double> {
  assert(segments.size() >= 2);

  std::vector<double> y_values;

  unsigned long segment = get_segment(x_values[0], segments);
  double m              = get_slope(segments[segment], segments[segment + 1]);
  for (auto x : x_values) {
    if (x > std::get<0>(segments[segment + 1])) {
      segment = get_segment(x, segments);
      m       = get_slope(segments[segment], segments[segment + 1]);
    }
    y_values.push_back(calc_interp_2d(std::get<0>(segments[segment]), std::get<1>(segments[segment]), m, x));
  }
  return y_values;
}

auto core::math::lin_interp::linspace(double start, double stop, std::size_t elems) -> std::vector<double> {
  double step = (stop - start) / static_cast<double>(elems - 1);
  std::vector<double> linsp(elems);
  std::vector<double>::iterator i;
  double val;
  for (i = linsp.begin(), val = start; i != linsp.end(); ++i, val += step) {
    *i = val;
  }
  return linsp;
}

int increase(int in) { return in + 1; }

#ifndef DOCTEST_CONFIG_DISABLE
// NOLINTBEGIN(*-magic-numbers)

#include <doctest/doctest.h>

#include <cmath>

TEST_SUITE("Linear interpolation") {
  TEST_CASE("Get slope") {
    std::tuple<double, double> p1(1.0, 1.0);
    std::tuple<double, double> p2(2.0, 2.0);

    CHECK_LE(std::abs(1.0 - get_slope(p1, p2)), 0.00000001);
  }
  TEST_CASE("Get segment") {
    std::vector<std::tuple<double, double>> segments = {std::tuple<double, double>(1.0, 1.0),
                                                        std::tuple<double, double>(2.0, 2.0),
                                                        std::tuple<double, double>(3.0, 4.0)};

    CHECK_EQ(get_segment(0.7, segments), 0);  // Before first segment, use first segment
    CHECK_EQ(get_segment(1.6, segments), 0);  // Within first segment
    CHECK_EQ(get_segment(2.4, segments), 1);  // Within last segment
    CHECK_EQ(get_segment(3.1, segments), 1);  // After last segment, use last segment
  }
  TEST_CASE("Calculate interpolation") {
    double x1 = 1.0;
    double y1 = 2.0;
    double m  = 1.0;
    double x  = 1.8;

    CHECK_LE(std::abs(2.8 - calc_interp_2d(x1, y1, m, x)), 0.00000001);
  }
  TEST_CASE("Interpolate points strictly within segments") {
    std::vector<std::tuple<double, double>> segments = {std::tuple<double, double>(1.0, 1.0),
                                                        std::tuple<double, double>(2.0, 2.0),
                                                        std::tuple<double, double>(3.0, 4.0)};
    std::vector<double> x_values                     = {1.1, 1.4, 1.7, 2.2, 2.5, 2.8};

    auto y_values = core::math::lin_interp::lin_interp_2d(x_values, segments);

    CHECK_EQ(y_values.size(), 6);
    CHECK_LE(std::abs(y_values[0] - 1.1), 0.00000001);
    CHECK_LE(std::abs(y_values[1] - 1.4), 0.00000001);
    CHECK_LE(std::abs(y_values[2] - 1.7), 0.00000001);
    CHECK_LE(std::abs(y_values[3] - 2.4), 0.00000001);
    CHECK_LE(std::abs(y_values[4] - 3.0), 0.00000001);
    CHECK_LE(std::abs(y_values[5] - 3.6), 0.00000001);
  }
  TEST_CASE("Interpolate points with extrapolation outside segments") {
    std::vector<std::tuple<double, double>> segments = {std::tuple<double, double>(1.0, 1.0),
                                                        std::tuple<double, double>(2.0, 2.0),
                                                        std::tuple<double, double>(3.0, 4.0)};
    std::vector<double> x_values                     = {0.7, 1.4, 1.7, 2.2, 2.5, 3.4};

    auto y_values = core::math::lin_interp::lin_interp_2d(x_values, segments);

    CHECK_EQ(y_values.size(), 6);
    CHECK_LE(std::abs(y_values[0] - 0.7), 0.00000001);
    CHECK_LE(std::abs(y_values[1] - 1.4), 0.00000001);
    CHECK_LE(std::abs(y_values[2] - 1.7), 0.00000001);
    CHECK_LE(std::abs(y_values[3] - 2.4), 0.00000001);
    CHECK_LE(std::abs(y_values[4] - 3.0), 0.00000001);
    CHECK_LE(std::abs(y_values[5] - 4.8), 0.00000001);
  }
}

// NOLINTEND(*-magic-numbers)
#endif