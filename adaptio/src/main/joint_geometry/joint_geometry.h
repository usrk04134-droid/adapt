#pragma once

#include <string>

namespace joint_geometry {

enum class WeldingType {
  LONGITUDINAL,
  CIRCUMFERENTIAL
};

auto WeldingTypeToString(WeldingType type) -> std::string;
auto WeldingTypeFromString(const std::string& str) -> WeldingType;

struct JointGeometry {
  double upper_joint_width_mm;
  double groove_depth_mm;
  double left_joint_angle_rad;
  double right_joint_angle_rad;
  double left_max_surface_angle_rad;
  double right_max_surface_angle_rad;
  WeldingType welding_type;

  auto ToString() const -> std::string;
};

}  // namespace joint_geometry
