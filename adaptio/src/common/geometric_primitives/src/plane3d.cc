
#include "plane3d.h"

#include <Eigen/Eigen>

#include "common/geometric_primitives/src/coordinate-systems.h"
#include "common/geometric_primitives/src/line3d.h"
#include "point3d.h"

namespace geometric_primitives {

Plane3d::Plane3d(Eigen::Vector3d normal, const Point3d &point_in_plane)
    : normal_(normal), point_in_plane_(point_in_plane) {
  normal_.normalize();
}

auto Plane3d::GetNormal() const -> Eigen::Vector3d { return this->normal_; }

auto Plane3d::GetHomNormal() const -> Eigen::Vector4d { return {normal_(0), normal_(1), normal_(2), 1.0}; }

auto Plane3d::GetPointInPlane() const -> Point3d { return this->point_in_plane_; }

auto Plane3d::GetRefSystem() const -> CoordinateSystem { return this->point_in_plane_.GetRefSystem(); }

auto Plane3d::Intersect(const Plane3d &other) const -> Line3d {
  const Eigen::Vector3d n1 = this->normal_;
  const Eigen::Vector3d n2 = other.GetNormal();
  const double h1          = n1.dot(this->point_in_plane_.ToVec());
  const double h2          = n2.dot(other.GetPointInPlane().ToVec());
  const double c1          = (h1 - h2 * n1.dot(n2)) / (1 - n1.dot(n2) * n1.dot(n2));
  const double c2          = (h2 - h1 * n1.dot(n2)) / (1 - n1.dot(n2) * n1.dot(n2));

  Eigen::Vector3d point_on_line     = c1 * n1 + c2 * n2;
  Eigen::Vector3d direction_of_line = n1.cross(n2);

  Line3d intersection_line{direction_of_line, Point3d::FromVector(point_on_line), 1.0};

  return intersection_line;
}

}  // namespace geometric_primitives
