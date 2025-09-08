#pragma once

#include <Eigen/Eigen>
#include <iostream>

namespace core::math::filter {

auto Median(const Eigen::RowVectorXd& source, size_t window_size) -> Eigen::RowVectorXd;

/*!
 * Applies a uniform filter with a selected window size using reflect strategy. The uniform filter is the arithmetic
 * average of the window size indices surrounding each indice in the input vector. The reflect strategy means that the
 * input vector is reflected where the window is outside the input vector bounds. E.g.
 * input vector: 2, 4, 3, 5, 1, 3, 2, 6
 * window size: 5
 * reflected vector: "4, 2," 2, 4, 3, 5, 1, 3, 2, 6, "6, 2"
 *
 * @param input Input vector.
 * @param window_size Size of the window.
 * @return Vector of the same dimension as the input vector.
 */
auto Uniform1dReflect(const std::vector<double>& input, size_t window_size) -> std::vector<double>;

}  // namespace core::math::filter