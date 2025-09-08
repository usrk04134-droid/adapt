#pragma once

#include "arc2d.h"
#include "intersection-point.h"
namespace deposition_simulator {

class Bead {
 private:
  Arc2d arc_;
  IntersectionPoint start_;
  IntersectionPoint end_;

 public:
  Bead();
  Bead(const Arc2d &arc, IntersectionPoint &start, IntersectionPoint &end);
  ~Bead() = default;
  auto GetStart() const -> IntersectionPoint;
  auto GetEnd() const -> IntersectionPoint;
  auto GetArc() const -> Arc2d;
};

}  // namespace deposition_simulator
