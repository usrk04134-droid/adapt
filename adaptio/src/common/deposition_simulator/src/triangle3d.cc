#include "../triangle3d.h"

#include <fmt/core.h>

#include <string>

#include "point3d.h"

namespace deposition_simulator {

Triangle3d::Triangle3d(Point3d& vertex1, Point3d& vertex2, Point3d& vertex3)
    : vertex1_(vertex1), vertex2_(vertex2), vertex3_(vertex3) {}

auto Triangle3d::ToString() const -> std::string {
  return fmt::format("Vertices: [{}, {}, {}]", vertex1_.ToString(), vertex2_.ToString(), vertex3_.ToString());
}

}  // Namespace deposition_simulator
