#pragma once

#include <cmath>
#include <vector>

#include "macs/macs_point.h"

namespace groove {

inline auto PolygonArea(const std::vector<macs::Point>& vec) -> double {
  double area = 0.;
  for (auto i = 0; i < vec.size(); i++) {
    auto const jj  = (i == 0) ? vec.size() - 1 : (i - 1);
    area          += vec[jj].horizontal * vec[i].vertical - vec[i].horizontal * vec[jj].vertical;
  }

  return std::fabs(area / 2.);
}

};  // namespace groove
