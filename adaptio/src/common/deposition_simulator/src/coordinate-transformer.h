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

const Eigen::Vector3d ROTATION_AXIS_ROCS = {1.0, 0.0, 0.0};
const Eigen::Vector3d ORIGIN             = {0.0, 0.0, 0.0};

class CoordinateTransformer {
 private:
  WeldMovementType weld_movement_type_{CIRCUMFERENTIAL};
  LpcsConfig lpcs_config_;
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
  CoordinateTransformer(LpcsConfig &lpcs_config, OpcsConfig &opcs_config, WeldMovementType weld_movement_type);
  ~CoordinateTransformer() = default;
  CoordinateTransformer();
  static auto CreateTransform(Vector3d &rot_ax, double rot_ang, Vector3d &translation, bool pure_rotation) -> Matrix4d;
  auto Transform(const Point3d &orig_point, CoordinateSystem target_system) const -> Point3d;
  auto Transform(const Plane3d &orig_plane, CoordinateSystem target_system) const -> Plane3d;
  auto ProjectToSlicePlane(const Point3d &point_any) const -> Point2d;
  auto DoWeldMovementProjectionToPlane(const Point3d &point_any, const Plane3d &plane_any, CoordinateSystem ref_system)
      -> std::optional<Point3d>;
  auto SetWeldObjectOrientation(Matrix4d &opcs_rel_to_rocs) -> void;
  auto SetTorchPos(Point3d &torchpos_any) -> void;
  auto GetTorchPos(CoordinateSystem target_system) const -> Point3d;
};

}  // namespace deposition_simulator
