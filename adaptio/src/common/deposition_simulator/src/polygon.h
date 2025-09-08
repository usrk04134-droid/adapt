#pragma once

#include <vector>

#include "line2d.h"
#include "point2d.h"

namespace deposition_simulator {

class Polygon {
 private:
  std::vector<Line2d> edges_;

 public:
  Polygon();
  ~Polygon() = default;
  auto Area() const -> double;
  auto AddEdge(const Line2d &edge_line) -> void;
  auto GetEdgePoints() const -> std::vector<Point2d>;
};

}  // namespace deposition_simulator
