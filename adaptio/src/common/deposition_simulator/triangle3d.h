#pragma once

#include <string>
#include <vector>

#include "point3d.h"

namespace deposition_simulator {

class Triangle3d {
 private:
  Point3d vertex1_;
  Point3d vertex2_;
  Point3d vertex3_;

 public:
  Triangle3d() = default;
  Triangle3d(Point3d& vertex1, Point3d& vertex2, Point3d& vertex3);
  auto ToString() const -> std::string;
};

}  // Namespace deposition_simulator
