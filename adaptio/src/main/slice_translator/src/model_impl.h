#pragma once

#include <Eigen/Eigen>

#include "common/geometric_primitives/src/circle3d.h"
#include "common/geometric_primitives/src/plane3d.h"
#include "common/geometric_primitives/src/point3d.h"
#include "common/groove/point.h"
#include "common/types/vector_3d.h"
#include "lpcs/lpcs_point.h"
#include "slice_translator/model_config.h"
#include "slice_translator/model_extract.h"
#include "slice_translator/slice_translator_service_v2.h"

namespace slice_translator {

using geometric_primitives::Circle3d;
using geometric_primitives::Plane3d;
using geometric_primitives::Point3d;

class ModelImpl : public SliceTranslatorServiceV2, public ModelConfig, public ModelExtract {
 public:
  /*SliceTranslatorServiceV2*/
  auto LPCSToMCS(const std::vector<lpcs::Point>& lpcs_points, const common::Point& slide_position) const
      -> std::optional<std::vector<common::Point>> override;
  auto MCSToLPCS(const std::vector<common::Point>& mcs_points, const common::Point& slide_position) const
      -> std::optional<std::vector<lpcs::Point>> override;
  auto DistanceFromTorchToScanner(const std::vector<lpcs::Point>& lpcs_points, const common::Point& axis_position) const
      -> std::optional<double> override;
  auto Available() const -> bool override;

  /*ModelConfig*/
  void Set(const common::Vector3D& rot_center, std::array<double, 3> scanner_angles,
           const common::Vector3D& weld_object_rotation_axis, const common::Vector3D& torch_to_laser_translation,
           double weld_object_radius) override;
  void Reset() override;

  /*ModelExtract*/
  auto TransformAndRotateToTorchPlane(const common::Vector3D& rot_center, std::array<double, 3> scanner_angles,
                                      const common::Vector3D& weld_object_rotation_axis,
                                      const common::Vector3D& torch_to_laser_translation, lpcs::Point point_lpcs,
                                      common::Point slide_position) const -> common::Point override;

  auto ComputeLpcsOrientation(double scanner_mount_angle, double delta_rot_y, double delta_rot_z) const
      -> Eigen::Matrix3d override;

 private:
  common::Vector3D rot_center_;
  double scanner_mount_angle_{0.0};
  double delta_rot_y_{0.0};
  double delta_rot_z_{0.0};
  common::Vector3D weld_object_rotation_axis_;
  common::Vector3D torch_to_laser_translation_;
  double weld_object_radius_{};
  bool available_ = false;

  auto AngleFromTorchToScanner(const std::vector<lpcs::Point>& lpcs_points, const common::Point& axis_position) const
      -> std::optional<double>;

  auto ComputeWeldObjectOrientation(const common::Vector3D& weld_object_rotation_axis) const -> Eigen::Matrix3d;
  auto CreateRotationMatrix(double angle, Eigen::Vector3d& rot_axis) const -> Eigen::Matrix3d;
  auto FindClosestPoint(std::vector<Point3d> point, Point3d ref_point) const -> Point3d;
  auto CreateProjectionCircle(const common::Vector3D& calib_rot_center,
                              const common::Vector3D& weld_object_rotation_axis, Point3d& point_in_plane) const
      -> Circle3d;
  auto RotateToPlane(const Circle3d& projection_circle, Plane3d& target_plane) const -> Point3d;
  auto TransformMACStoLPCS(std::array<double, 3> scanner_angles, const common::Vector3D& torch_to_laser_translation,
                           common::Point slide_position, Point3d point_macs, bool use_translation = true) const
      -> Point3d;
  auto TransformLPCStoMACS(std::array<double, 3> scanner_angles, const common::Vector3D& torch_to_laser_translation,
                           common::Point slide_position, Point3d point_lpcs, bool use_translation = true) const
      -> Point3d;
  auto TransformAndRotateToLaserPlane(const common::Vector3D& rot_center, std::array<double, 3> scanner_angles,
                                      const common::Vector3D& weld_object_rotation_axis,
                                      const common::Vector3D& torch_to_laser_translation, common::Point point_macs,
                                      common::Point slide_position) const -> lpcs::Point;
};

}  // namespace slice_translator
