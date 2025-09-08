#pragma once

#include <Eigen/Eigen>

#include "plane3d.h"
#include "point3d.h"

namespace geometric_primitives {

class Circle3d {
 private:
  Plane3d circle_plane_;
  Point3d center_;
  double radius_;
  auto CreateCircleBasis(Line3d &line_in_plane) const -> Eigen::Matrix3d;

 public:
  Circle3d(Eigen::Vector3d &plane_normal, double radius, Point3d &center);
  auto Intersect(const Plane3d &plane) const -> std::vector<Point3d>;
};

}  // Namespace geometric_primitives
