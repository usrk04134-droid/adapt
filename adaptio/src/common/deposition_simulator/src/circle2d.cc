
#include "circle2d.h"

#include <cmath>

#include "point2d.h"
namespace deposition_simulator {

Circle2d::Circle2d() : radius_(1) {}
// Circle2d::Circle2d(const Eigen::Vector2d& center, double radius)
// {

// }

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
Circle2d::Circle2d(double x_center, double y_center, double radius) : radius_(radius) {
  this->center_ = Point2d(x_center, y_center);
}

auto Circle2d::GetPoint(double angle) const -> Point2d {
  const double x_coord = (std::cos(angle) * this->radius_) + this->center_.GetX();
  const double y_coord = (std::sin(angle) * this->radius_) + this->center_.GetY();
  return {x_coord, y_coord};
}

auto Circle2d::GetCenter() const -> Point2d { return this->center_; }

auto Circle2d::GetRadius() const -> double { return this->radius_; }

}  // namespace deposition_simulator
