#pragma once

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>
#include <vector>

namespace core::math::value {

inline auto StandardDeviation(const std::vector<double>& v, long start_index, long stop_index) -> double {
  const long sz = stop_index - start_index;

  double sum  = std::accumulate(v.begin() + start_index, v.begin() + stop_index, 0.0);
  double mean = sum / sz;

  std::vector<double> diff(sz);
  std::transform(v.begin() + start_index, v.begin() + stop_index, diff.begin(), [mean](double x) { return x - mean; });

  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  double stdev  = std::sqrt(sq_sum / sz);

  return stdev;
}

auto FindMedian(std::random_access_iterator auto begin, std::random_access_iterator auto end, unsigned long size)
    -> double {
  // Size is even
  if (size % 2 == 0) {
    std::nth_element(begin, begin + size / 2, end);

    std::nth_element(begin, begin + (size - 1) / 2, end);

    return (*(begin + size / 2) + *(begin + (size - 1) / 2)) / 2.0;
  } else {
    std::nth_element(begin, begin + size / 2, end);

    return *(begin + size / 2);
  }
}

}  // namespace core::math::value
