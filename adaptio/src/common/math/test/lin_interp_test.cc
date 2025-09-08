#include "common/math/lin_interp.h"

#include <doctest/doctest.h>

#include <cmath>
#include <tuple>
#include <vector>

// NOLINTBEGIN(*-magic-numbers)
namespace common::math::lin_interp {

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

    auto y_values = lin_interp_2d(x_values, segments);

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

    auto y_values = lin_interp_2d(x_values, segments);

    CHECK_EQ(y_values.size(), 6);
    CHECK_LE(std::abs(y_values[0] - 0.7), 0.00000001);
    CHECK_LE(std::abs(y_values[1] - 1.4), 0.00000001);
    CHECK_LE(std::abs(y_values[2] - 1.7), 0.00000001);
    CHECK_LE(std::abs(y_values[3] - 2.4), 0.00000001);
    CHECK_LE(std::abs(y_values[4] - 3.0), 0.00000001);
    CHECK_LE(std::abs(y_values[5] - 4.8), 0.00000001);
  }
}
}  // namespace common::math::lin_interp
// NOLINTEND(*-magic-numbers)
