#pragma once

#include <Eigen/Eigen>
#include <memory>

#include "../point3d.h"

namespace deposition_simulator {

using Eigen::AngleAxisd;
using Eigen::Vector3d;

class Plane3d;

class Line3d {
 private:
  Point3d start_;
  Eigen::Vector3d direction_;
  double length_;
  Line3d(const Point3d &start, const Point3d &end);

 public:
  Line3d();
  Line3d(const Eigen::Vector3d &dir, const Point3d &start, double length);
  ~Line3d() = default;
  auto Reverse() -> void;
  auto Translate(const Vector3d &translation) -> void;
  auto Rotate(const AngleAxisd &axis_angle) -> void;
  auto GetRefSystem() const -> CoordinateSystem;
  auto GetEnd() const -> Point3d;
  auto GetStart() const -> Point3d;
  auto GetDirection() const -> Eigen::Vector3d;
  auto Length() const -> double;
  auto Intersect(const Plane3d &plane, bool consider_line_len) -> std::unique_ptr<Point3d>;
  static auto FromPoints(const Point3d &start, const Point3d &end) -> Line3d;
};

}  // namespace deposition_simulator
