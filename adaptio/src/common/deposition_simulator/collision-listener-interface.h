#pragma once

#include "point3d.h"
namespace deposition_simulator {

class ICollisionListener {
 public:
  virtual ~ICollisionListener()                             = 0;
  virtual auto OnTorchCollision(Point3d &hit_point) -> void = 0;
};

}  // Namespace deposition_simulator
