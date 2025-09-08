#pragma once

#include <Eigen/Eigen>

namespace deposition_simulator {

enum CoordinateSystem {
  CLCS = 0,
  MACS = 1,
  ROCS = 2,
  TCS  = 3,
  LPCS = 4,
  CCS  = 5,
  IPCS = 6,
  ICS  = 7,
  SCS  = 8,  // Slice Coorindate system (2D)
  OPCS = 9   // Object positioner
};

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
  auto ToString() const -> std::string;
  auto GetRefSystem() const -> CoordinateSystem;
  auto GetX() const -> double;
  auto GetY() const -> double;
  auto GetZ() const -> double;
};

}  // namespace deposition_simulator
