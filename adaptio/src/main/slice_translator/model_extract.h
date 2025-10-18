#pragma once

#include <Eigen/Eigen>

#include "calibration/src/calibration_solver.h"
#include "common/groove/point.h"
#include "lpcs/lpcs_point.h"

namespace slice_translator {

/*This interface is to be used to calculate or solve for the model parameters.
  It should only be used by the "calibration::WeldObjectCalibrationSolver" class*/

class ModelExtract {
 public:
  virtual ~ModelExtract() = default;
  virtual auto TransformAndRotateToTorchPlane(const common::Vector3D& rot_center, std::array<double, 3> scanner_angles,
                                              const common::Vector3D& weld_object_rotation_axis,
                                              const common::Vector3D& torch_to_laser_translation,
                                              lpcs::Point point_lpcs, common::Point slide_position) const
      -> common::Point = 0;

  virtual auto ComputeLpcsOrientation(double tilt_angle, double delta_rot_y, double delta_rot_z) const
      -> Eigen::Matrix3d = 0;
};

}  // namespace slice_translator
