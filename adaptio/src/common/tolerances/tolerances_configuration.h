#pragma once

#include <fmt/core.h>

#include <string>

namespace tolerances {

struct Configuration {
  struct {
    double upper_width;
    double surface_angle;
    double wall_angle;
  } joint_geometry;
};

inline auto ToString(const Configuration& to) -> std::string {
  return fmt::format("{{joint_geometry = [upper_width={:.1f}, surface_angle={}, wall_angle={}]}}",
                     to.joint_geometry.upper_width, to.joint_geometry.surface_angle, to.joint_geometry.wall_angle);
}

}  // namespace tolerances
