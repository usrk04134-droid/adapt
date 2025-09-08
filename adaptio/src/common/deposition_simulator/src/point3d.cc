
#include "../point3d.h"

#include <Eigen/src/Core/Matrix.h>
#include <fmt/core.h>

#include <Eigen/Eigen>
#include <string>

using Eigen::Vector3d;

namespace deposition_simulator {

// NOLINENEXTLINE(bugprone-easily-swappable-parameters)
Point3d::Point3d(double x_coord, double y_coord, double z_coord, CoordinateSystem ref)
    : x_(x_coord), y_(y_coord), z_(z_coord), ref_system_(ref) {}

Point3d::Point3d() : x_(0.0), y_(0.0), z_(0.0), ref_system_(MACS) {}

auto Point3d::GetRefSystem() const -> CoordinateSystem { return this->ref_system_; }
auto Point3d::GetX() const -> double { return this->x_; }
auto Point3d::GetY() const -> double { return this->y_; }
auto Point3d::GetZ() const -> double { return this->z_; }
// Point3d Point3d::NewByTranslation(Eigen::Vector3d transVec)
// {
//   Point3d p;
//   //TODO: create and return then new point
//   return p;
// }

auto Point3d::operator+(Vector3d &other) const -> Point3d {
  const Point3d new_point(this->x_ + other(0), this->y_ + other(1), this->z_ + other(2), this->ref_system_);
  // newPoint.x = this->x + other(0);
  // newPoint.y = this->y + other(1);
  // newPoint.z = this->z + other(2);
  return new_point;
}

auto Point3d::ToHomVec() const -> Eigen::Vector4d { return {this->x_, this->y_, this->z_, 1.0}; }
auto Point3d::ToVec() const -> Eigen::Vector3d { return {this->x_, this->y_, this->z_}; }
auto Point3d::ToString() const -> std::string { return fmt::format("{}, {}, {}", x_, y_, z_); }

}  // namespace deposition_simulator
