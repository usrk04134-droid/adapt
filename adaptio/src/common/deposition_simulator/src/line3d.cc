
#include "line3d.h"

#include <cmath>
#include <Eigen/Eigen>
#include <memory>
#include <stdexcept>

#include "../point3d.h"
#include "plane3d.h"

namespace deposition_simulator {

const double DOT_PRODUCT_PARALLEL = 1e-12;
// const double MIN_LINE_LENGTH      = 1e-6;

Line3d::Line3d(const Point3d &start, const Point3d &end) : start_(start), direction_({0.0, 0.0, 0.0}), length_(0) {
  Vector3d dir(end.GetX() - start.GetX(), end.GetY() - start.GetY(), end.GetZ() - start.GetZ());
  this->length_ = dir.norm();
  dir.normalize();
  this->direction_ = dir;
  // this->start_     = start;
}

Line3d::Line3d() : length_(0) {}

auto Line3d::Reverse() -> void {
  const Point3d new_start = this->GetEnd();
  this->direction_        = -this->direction_;
  this->start_            = new_start;
}

auto Line3d::GetRefSystem() const -> CoordinateSystem { return this->start_.GetRefSystem(); }

auto Line3d::Translate(const Vector3d &translation) -> void {}

auto Line3d::Rotate(const AngleAxisd &axis_angle) -> void {}

auto Line3d::Intersect(const Plane3d &plane, bool consider_line_len) -> std::unique_ptr<Point3d> {
  if (plane.GetRefSystem() != this->GetRefSystem()) {
    throw std::runtime_error("Cannot intersect plane and line defined in different coordinate systems.");
  }

  const Eigen::Vector3d normal   = plane.GetNormal();
  const Eigen::Vector3d p_plane  = plane.GetPointInPlane().ToVec();
  const Eigen::Vector3d p_line   = this->start_.ToVec();
  const Eigen::Vector3d line_vec = this->direction_;

  if (std::abs(line_vec.dot(normal)) < DOT_PRODUCT_PARALLEL) {  // Line (near) parallell with plane
    return nullptr;
  }

  const double dist_along_line = normal.dot(p_plane - p_line) / line_vec.dot(normal);
  Eigen::Vector3d p_int        = p_line + line_vec * dist_along_line;

  if (consider_line_len) {
    if (dist_along_line < 0 || dist_along_line > this->length_) {
      // std::cout << "Filtering plane line intersection. (d, len) " << distAlongLine << ", " << this->length <<
      // std::endl;
      return nullptr;
    }
  }

  // std::cout << "Found plane line intersection. (d, len) " << distAlongLine << ", " << this->length << std::endl;

  return std::make_unique<Point3d>(Point3d(p_int(0), p_int(1), p_int(2), this->GetRefSystem()));
}

auto Line3d::GetEnd() const -> Point3d {
  Vector3d vec = this->direction_ * this->length_;
  return this->start_ + vec;
}

auto Line3d::GetStart() const -> Point3d { return this->start_; }

auto Line3d::FromPoints(const Point3d &start, const Point3d &end) -> Line3d { return {start, end}; }

}  // namespace deposition_simulator
