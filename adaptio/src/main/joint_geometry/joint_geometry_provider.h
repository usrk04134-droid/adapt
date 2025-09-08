#pragma once

#include <functional>
#include <optional>

#include "joint_geometry/joint_geometry.h"

namespace joint_geometry {

class JointGeometryProvider {
 public:
  JointGeometryProvider()          = default;
  virtual ~JointGeometryProvider() = default;

  virtual auto GetFixtureJointGeometry() const -> joint_geometry::JointGeometry         = 0;
  virtual auto GetJointGeometry() const -> std::optional<joint_geometry::JointGeometry> = 0;
  virtual void Subscribe(std::function<void()> on_update)                               = 0;
};

}  // namespace joint_geometry
