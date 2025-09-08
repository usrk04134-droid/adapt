
#include "polygon.h"

#include <Eigen/src/Core/Matrix.h>

#include <vector>

#include "line2d.h"

namespace deposition_simulator {

Polygon::Polygon() = default;

auto Polygon::AddEdge(const Line2d &edge_line) -> void { this->edges_.push_back(edge_line); }

auto Polygon::Area() const -> double {
  double double_area = 0.0;
  Eigen::Matrix<double, 2, 2> shoelace_matrix;

  for (const Line2d &line : this->edges_) {
    shoelace_matrix(0, 0)  = line.GetStart().GetX();
    shoelace_matrix(0, 1)  = line.GetEnd().GetX();
    shoelace_matrix(1, 0)  = line.GetStart().GetY();
    shoelace_matrix(1, 1)  = line.GetEnd().GetY();
    double_area           += shoelace_matrix.determinant();
  }

  return double_area / 2;
}

auto Polygon::GetEdgePoints() const -> std::vector<Point2d> {
  std::vector<Point2d> abw;
  abw.reserve(this->edges_.size());

  for (const auto &edge : this->edges_) {
    abw.push_back(edge.GetStart());
  }

  return abw;
}

}  // namespace deposition_simulator
