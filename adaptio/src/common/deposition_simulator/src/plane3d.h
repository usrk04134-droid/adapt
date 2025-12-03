#pragma once

#include <Eigen/Eigen>
#include <string>

#include "../point3d.h"
#include "line3d.h"

namespace deposition_simulator {
class Plane3d {
 private:
  Eigen::Vector3d normal_;
  Point3d point_in_plane_;

 public:
  Plane3d(Eigen::Vector3d normal, const Point3d &point_in_plane);
  ~Plane3d() = default;
  auto GetNormal() const -> Eigen::Vector3d;
  auto GetHomNormal() const -> Eigen::Vector4d;
  auto GetPointInPlane() const -> Point3d;
  auto GetRefSystem() const -> CoordinateSystem;
  auto Intersect(const Plane3d &other) const -> Line3d;
};

}  // namespace deposition_simulator
