#pragma once

#include <optional>

#include "common/types/vector_3d.h"
#include "lpcs/lpcs_point.h"

namespace slice_translator {

class ModelExtract {
 public:
  virtual ~ModelExtract() = default;

  virtual auto TransformAndRotateToTorchPlane(const common::Vector3D& rot_center, std::array<double, 3> scanner_angles,
                                              const common::Vector3D& weld_object_rotation_axis,
                                              const common::Vector3D& torch_to_laser_translation, lpcs::Point point_lpcs,
                                              common::groove::Point slide_position) const -> macs::Point = 0;

  virtual auto ComputeLpcsOrientation(double scanner_mount_angle, double delta_rot_y, double delta_rot_z) const
      -> Eigen::Matrix3d = 0;
};

}  // namespace slice_translator
