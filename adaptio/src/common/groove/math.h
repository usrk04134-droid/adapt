#pragma once

#include <cmath>
#include <vector>

#include "point.h"

namespace common::groove {

inline auto PolygonArea(const std::vector<Point>& vec) -> double {
  double area = 0.;
  for (auto i = 0; i < static_cast<int>(vec.size()); i++) {
    auto const jj = (i == 0) ? static_cast<int>(vec.size()) - 1 : (i - 1);
    area         += vec[jj].horizontal * vec[i].vertical - vec[i].horizontal * vec[jj].vertical;
  }

  return std::fabs(area / 2.);
}

}  // namespace common::groove

