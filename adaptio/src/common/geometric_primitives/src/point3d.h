#pragma once

#include <Eigen/Eigen>

#include "common/geometric_primitives/src/coordinate-systems.h"

namespace geometric_primitives {

class Point3d {
 private:
  double x_;
  double y_;
  double z_;
  CoordinateSystem ref_system_;

 public:
  // NOLINTNEXTLINE(readability-identifier-length)
  Point3d(double x_coord, double y_coord, double z_coord, CoordinateSystem ref);
  Point3d();
  ~Point3d() = default;
  auto operator+(Eigen::Vector3d &other) const -> Point3d;
  auto ToHomVec() const -> Eigen::Vector4d;
  auto ToVec() const -> Eigen::Vector3d;
  auto GetRefSystem() const -> CoordinateSystem;
  auto GetX() const -> double;
  auto GetY() const -> double;
  auto GetZ() const -> double;
  auto DistanceTo(Point3d &ref_point) const -> double;
  auto static FromVector(Eigen::Vector3d &vec) -> Point3d;
};

}  // namespace geometric_primitives
