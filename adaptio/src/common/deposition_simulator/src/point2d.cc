
#include "point2d.h"

#include <Eigen/src/Core/Matrix.h>

#include <cmath>
#include <numbers>

namespace deposition_simulator {

Point2d::Point2d() : x_(0), y_(0) {}

// NOLINENEXTLINE(bugprone-easily-swappable-parameters)
Point2d::Point2d(double x_coord, double y_coord) : x_(x_coord), y_(y_coord) {}

auto Point2d::operator+(const Eigen::Vector2d &other) const -> Point2d {
  Point2d new_point;
  new_point.x_ = this->x_ + other(0);
  new_point.y_ = this->y_ + other(1);
  return new_point;
}

auto Point2d::GetX() const -> double { return this->x_; }

auto Point2d::GetY() const -> double { return this->y_; }

auto Point2d::GetPolarAngle(const Point2d &ref, bool range_2pi) const -> double {
  const double ang = std::atan2(this->GetY() - ref.GetY(), this->GetX() - ref.GetX());

  return (ang < 0 && range_2pi) ? (2 * std::numbers::pi) + ang : ang;
}

auto Point2d::ToVector() const -> Eigen::Vector2d { return {this->GetX(), this->GetY()}; }

}  // namespace deposition_simulator
