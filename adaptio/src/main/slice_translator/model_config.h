#pragma once

#include <common/types/vector_3d.h>

#include <array>

namespace slice_translator {

/*This interface is to be used to set model parameters as precondition for
  joint tracking or automatic welding.
  The interface should only be used by the "calibration::CalibrationManager" class*/

class ModelConfig {
 public:
  virtual ~ModelConfig() = default;

  virtual auto Set(const common::Vector3D& rot_center, std::array<double, 3> scanner_angles,
                   const common::Vector3D& weld_object_rotation_axis,
                   const common::Vector3D& torch_to_laser_translation, double weld_object_radius) -> void = 0;
  virtual void Reset()                                                                                    = 0;
};

}  // namespace slice_translator
