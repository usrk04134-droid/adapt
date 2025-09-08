#include "common/math/lin_interp.h"

#include <assert.h>

#include <cstddef>
#include <tuple>
#include <vector>

namespace common::math::lin_interp {

auto get_slope(const std::tuple<double, double> &p1, const std::tuple<double, double> &p2) -> double {
  return (std::get<1>(p2) - std::get<1>(p1)) / (std::get<0>(p2) - std::get<0>(p1));
}

auto calc_interp_2d(double x1, double y1, double m, double x) -> double { return y1 + m * (x - x1); }

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

auto lin_interp_2d(const std::vector<double> &x_values, const std::vector<std::tuple<double, double>> &segments)
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

auto linspace(double start, double stop, std::size_t elems) -> std::vector<double> {
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

}  // namespace common::math::lin_interp
