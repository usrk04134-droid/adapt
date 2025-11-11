#pragma once

#include <cmath>
#include <vector>

#include "common/groove/point.h"

namespace common {

inline auto PolygonArea(const std::vector<common::Point>& vec) -> double {
  double area = 0.;
  for (auto i = 0; i < vec.size(); i++) {
    auto const jj  = (i == 0) ? vec.size() - 1 : (i - 1);
    area          += vec[jj].horizontal * vec[i].vertical - vec[i].horizontal * vec[jj].vertical;
  }

  return std::fabs(area / 2.);
}

};  // namespace common
