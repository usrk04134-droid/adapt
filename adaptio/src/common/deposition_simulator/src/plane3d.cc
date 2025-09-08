
#include "plane3d.h"

#include "../point3d.h"
namespace deposition_simulator {

Plane3d::Plane3d(Eigen::Vector3d normal, const Point3d &point_in_plane)
    : normal_(normal), point_in_plane_(point_in_plane) {
  normal.normalize();
}

auto Plane3d::GetNormal() const -> Eigen::Vector3d { return this->normal_; }

auto Plane3d::GetHomNormal() const -> Eigen::Vector4d { return {normal_(0), normal_(1), normal_(2), 1.0}; }

auto Plane3d::GetPointInPlane() const -> Point3d { return this->point_in_plane_; }

auto Plane3d::GetRefSystem() const -> CoordinateSystem { return this->point_in_plane_.GetRefSystem(); }

}  // namespace deposition_simulator
