#pragma once

#include <Eigen/Eigen>

#include "../point3d.h"
#include "../sim-config.h"
#include "plane3d.h"
#include "point2d.h"

namespace deposition_simulator {

using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Transform;
using Eigen::Vector3d;
using Eigen::Vector4d;

class CoordinateTransformer {
 private:
  LpcsConfig lpcs_config_;
  // MacsConfig macs_config_;
  OpcsConfig opcs_config_;
  Matrix4d opcs_rel_to_rocs_;  // M_2->9
  Vector3d torchpos_macs_;
  auto InternalTransform(Vector4d &vec, CoordinateSystem orig_system, CoordinateSystem target_system,
                         bool pure_rotation) const -> Vector3d;
  // auto GetMacsToClcsTransform() const -> Matrix4d;                    // Transform from MACS to CLCS
  auto GetMacsToRocsTransform(bool pure_rotation) const -> Matrix4d;  // Transform from MACS to ROCS
  auto GetMacsToLpcsTransform(bool pure_rotation) const -> Matrix4d;  // Transform from MACS to LPCS
  auto GetMacsToTcsTransform(bool pure_rotation) const -> Matrix4d;   // Transform from MACS to TCS
  auto GetMacsToOpcsTransform(bool pure_rotation) const -> Matrix4d;  // Transform from MACS to RBCS

 public:
  CoordinateTransformer(LpcsConfig &lpcs_config, OpcsConfig &opcs_config);
  ~CoordinateTransformer() = default;
  CoordinateTransformer();
  static auto CreateTransform(Vector3d &rot_ax, double rot_ang, Vector3d &translation, bool pure_rotation) -> Matrix4d;
  auto Transform(const Point3d &orig_point, CoordinateSystem target_system) const -> Point3d;
  auto Transform(const Plane3d &orig_plane, CoordinateSystem target_system) const -> Plane3d;
  auto ProjectToSlicePlane(const Point3d &point_any) const -> Point2d;
  // auto IncrWeldObjectRotation(double delta_angle) -> double;
  auto SetWeldObjectOrientation(Matrix4d &opcs_rel_to_rocs) -> void;
  // auto GetWeldObjectRotationAngle() const -> double;
  auto SetTorchPos(Point3d &torchpos_any) -> void;
  auto GetTorchPos(CoordinateSystem target_system) const -> Point3d;
  // auto GetTorchClockAngle() const -> double;
  // auto GetTorchPlaneNormal() const -> Eigen::Vector3d;
};

}  // namespace deposition_simulator
