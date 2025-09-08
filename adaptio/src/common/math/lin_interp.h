#pragma once

#include <tuple>
#include <vector>

namespace common::math::lin_interp {
/*!
 * Interpolates y values for the input x values over a set of continuous line segments. The x input values must be
 * strictly increasing, if they are not the resulting y values are undefined.
 *
 * @param x_values  A strictly increasing set of x values.
 * @param segments  A set of coordinates that forms line segments, must be at least two coordinates.
 * @return y values corresponding to the input x points
 */
auto lin_interp_2d(const std::vector<double> &x_values, const std::vector<std::tuple<double, double>> &segments)
    -> std::vector<double>;

/*!
 * Creates a sequence of linearly spaced number over a given interval.
 * @param start Start of interval.
 * @param stop End of interval/
 * @param elems number of elements in the interval.
 * @return Vector the size of elems.
 */
auto linspace(double start, double stop, std::size_t elems) -> std::vector<double>;

/* for test */
auto get_slope(const std::tuple<double, double> &p1, const std::tuple<double, double> &p2) -> double;
auto get_segment(double x_value, const std::vector<std::tuple<double, double>> &segments) -> unsigned long;
auto calc_interp_2d(double x1, double y1, double m, double x) -> double;

}  // namespace common::math::lin_interp
