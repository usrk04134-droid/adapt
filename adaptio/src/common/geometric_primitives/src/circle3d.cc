#include "circle3d.h"

#include <Eigen/Eigen>
#include <vector>

#include "common/geometric_primitives/src/circle2d.h"
#include "common/geometric_primitives/src/line2d.h"
#include "common/geometric_primitives/src/line3d.h"
#include "common/geometric_primitives/src/plane3d.h"
#include "common/geometric_primitives/src/point3d.h"

namespace geometric_primitives {

Circle3d::Circle3d(Eigen::Vector3d &plane_normal, double radius, Point3d &center)
    : circle_plane_{plane_normal, center}, center_(center), radius_(radius) {}

auto Circle3d::Intersect(const Plane3d &plane) const -> std::vector<Point3d> {
  // Intersection line between circle plane and other plane. By def. in both planes
  Line3d int_line              = this->circle_plane_.Intersect(plane);
  Eigen::Matrix3d circle_basis = CreateCircleBasis(int_line);  // 1st basis vector: plane normal
  Eigen::Vector3d dir_b        = circle_basis.transpose() * int_line.GetDirection();
  Eigen::Vector3d point_b      = circle_basis.transpose() * (int_line.GetStart().ToVec() - this->center_.ToVec());

  Line2d int_line_b{
      {point_b(1), point_b(2)},
      {dir_b(1),   dir_b(2)  },
      1.0
  };
  Circle2d circle_b{0.0, 0.0, this->radius_};
  std::vector<Point2d> int_points_b = int_line_b.Intersect(circle_b, false, false);
  Eigen::Vector3d tmp_b;
  Eigen::Vector3d tmp;

  std::vector<Point3d> int_points_3d;
  for (auto const &int_point_b : int_points_b) {
    tmp_b = {0.0, int_point_b.GetX(), int_point_b.GetY()};
    tmp   = circle_basis * tmp_b + this->center_.ToVec();
    int_points_3d.emplace_back(tmp(0), tmp(1), tmp(2), plane.GetRefSystem());
  }

  return int_points_3d;
}

// Computes a basis with the two axes in the circle plane and with
// the third axis equal to the plane normal.
auto Circle3d::CreateCircleBasis(Line3d &line_in_plane) const -> Eigen::Matrix3d {
  Eigen::Matrix3d basis;
  basis.block(0, 0, 3, 1) = this->circle_plane_.GetNormal();
  basis.block(0, 1, 3, 1) = line_in_plane.GetDirection();
  basis.block(0, 2, 3, 1) = this->circle_plane_.GetNormal().cross(line_in_plane.GetDirection());

  return basis;
}

}  // Namespace geometric_primitives
