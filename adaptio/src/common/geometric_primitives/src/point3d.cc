#include "point3d.h"

#include <Eigen/Eigen>

#include "common/geometric_primitives/src/coordinate-systems.h"

using Eigen::Vector3d;

namespace geometric_primitives {

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

auto Point3d::DistanceTo(Point3d &ref_point) const -> double {
  double dist = (this->ToVec() - ref_point.ToVec()).norm();
  return dist;
}

auto Point3d::ToHomVec() const -> Eigen::Vector4d { return {this->x_, this->y_, this->z_, 1.0}; }

auto Point3d::ToVec() const -> Eigen::Vector3d { return {this->x_, this->y_, this->z_}; }

auto Point3d::FromVector(Eigen::Vector3d &vec) -> Point3d { return {vec(0), vec(1), vec(2), UNKNOWN}; }

}  // namespace geometric_primitives
