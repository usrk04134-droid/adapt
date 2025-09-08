#pragma once

#include <string>

namespace joint_geometry {

struct JointGeometry {
  double upper_joint_width_mm;
  double groove_depth_mm;
  double left_joint_angle_rad;
  double right_joint_angle_rad;
  double left_max_surface_angle_rad;
  double right_max_surface_angle_rad;

  auto ToString() const -> std::string;
};

}  // namespace joint_geometry
