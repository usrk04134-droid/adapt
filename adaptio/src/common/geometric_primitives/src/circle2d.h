#pragma once

#include <Eigen/Eigen>

#include "point2d.h"

namespace geometric_primitives {

class Circle2d {
 private:
  double radius_;
  Point2d center_;

 public:
  Circle2d();
  // Circle2d(const Eigen::Vector2d& center, double radius);
  Circle2d(double x_center, double y_center, double radius);
  ~Circle2d() = default;
  auto GetPoint(double angle) const -> Point2d;
  auto GetCenter() const -> Point2d;
  auto GetRadius() const -> double;
};
}  // namespace geometric_primitives
