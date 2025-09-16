#pragma once

#include <common/types/vector_3d.h>

#include <optional>
#include <vector>

#include "calibration/calibration_types.h"
#include "lpcs/lpcs_point.h"
#include "macs/macs_point.h"

namespace calibration {

struct Observation {
  macs::Point slide_position;
  std::vector<lpcs::Point> abw_points_lpcs;
};

struct TorchPlaneInfo {
  macs::Point top_center_at_torch_plane;
  macs::Point bottom_center_at_torch_plane;
};

struct GeometricConstants {
  double object_radius;
  double scanner_mount_angle;
  double ltc_stickout;
  double ltc_laser_plane_distance;
};

struct CalibrationResult {
  common::Vector3D weld_object_rotation_axis;
  common::Vector3D rotation_center;
  common::Vector3D torch_to_lpcs_translation;
  double delta_rot_y;
  double delta_rot_z;

  // Quality metrics
  macs::Point torch_plane_reference;
  std::vector<macs::Point> projected_points;
  double max_residual;
  double standard_deviation;
  double residual_sum_of_squares;
  double residual_standard_error;
  double r_squared;
  double adjusted_r_squared;
};

class CalibrationSolver {
 public:
  virtual ~CalibrationSolver() = default;

  virtual auto Calculate(const TorchPlaneInfo &torch_plane_info, const GeometricConstants &geometric_constants,
                         const std::vector<calibration::Observation> &observations)
      -> std::optional<CalibrationResult> = 0;
};

}  // namespace calibration

