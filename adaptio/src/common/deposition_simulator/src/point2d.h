#pragma once

#include <Eigen/Eigen>

namespace deposition_simulator {

class Point2d {
 private:
  double x_;
  double y_;

 public:
  auto GetX() const -> double;
  auto GetY() const -> double;
  auto GetPolarAngle(const Point2d &ref, bool range_2pi) const -> double;
  auto ToVector() const -> Eigen::Vector2d;
  Point2d();
  Point2d(double x_coord, double y_coord);
  ~Point2d() = default;
  auto operator+(const Eigen::Vector2d &other) const -> Point2d;
};

}  // namespace deposition_simulator
