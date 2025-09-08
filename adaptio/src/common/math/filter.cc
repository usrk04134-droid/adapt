#include "common/math/filter.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <Eigen/Core>
#include <vector>

namespace common::math::filter {
auto Median(const Eigen::RowVectorXd& source, size_t window_size) -> Eigen::RowVectorXd {
  using Eigen::Index;
  using Eigen::RowVectorXd;

  RowVectorXd target = RowVectorXd::Zero(1, source.cols());

  if (window_size % 2 == 0) {
    window_size = window_size - 1;
  }

  if (source.size() <= window_size) {
    return source;
  }

  std::vector<double> window(window_size);

  auto start = static_cast<int64_t>(window.size()) / 2;

  for (int64_t i = 0; i < start; i++) {
    target(i) = source(i);
  }
  for (int64_t i = start; i < static_cast<int64_t>(source.size()) - start - 1; i++) {
    const int64_t window_source_min = i - start;

    for (int64_t j = 0; j < window_size; j++) {
      const int64_t index = j + window_source_min;
      window[j]           = source[index];
    }

    std::sort(window.begin(), window.end());

    target(i) = window[window_size / 2];
  }
  for (int64_t i = static_cast<int64_t>(source.size()) - start - 1; i < static_cast<int64_t>(source.size()); i++) {
    target(i) = source(i);
  }

  return target;
}

auto Uniform1dReflect(const std::vector<double>& input, size_t window_size) -> std::vector<double> {
  std::vector<double> output;

  if (input.size() < window_size) {
    return input;
  }

  // Use an uneven window
  auto actual_window_size = window_size;
  if (actual_window_size % 2 == 0) {
    actual_window_size -= 1;
  }
  int half_window = static_cast<int>((actual_window_size - 1) / 2);
  auto low_ext    = std::vector<double>(input.begin(), input.begin() + half_window);
  std::reverse(low_ext.begin(), low_ext.end());
  auto high_ext = std::vector<double>(input.end() - half_window, input.end());
  std::reverse(high_ext.begin(), high_ext.end());

  auto uniform_vector = low_ext;
  uniform_vector.insert(uniform_vector.end(), input.begin(), input.end());
  uniform_vector.insert(uniform_vector.end(), high_ext.begin(), high_ext.end());

  for (auto i = 0; i < input.size(); i++) {
    auto uniform = 0.0;
    auto n       = 0;
    for (auto j = 0; j < actual_window_size && i + j < uniform_vector.size(); j++) {
      uniform += uniform_vector[i + j];
      n++;
    }
    if (n > 0) {
      output.push_back(uniform / static_cast<double>(n));
    } else {
      output.push_back(0.0);
    }
  }

  return output;
}
}  // namespace common::math::filter
