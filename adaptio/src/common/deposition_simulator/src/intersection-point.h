#pragma once

#include "line2d.h"
#include "point2d.h"

namespace deposition_simulator {

class IntersectionPoint : public Point2d {
 private:
  Line2d* intersectedLine_;
  // double x_;
  // double y_;

 public:
  IntersectionPoint();
  IntersectionPoint(double x_coord, double y_coord, Line2d* line);  // NOLINT(readability-identifier-length)
  ~IntersectionPoint() = default;
  auto GetIntersectedLine() const -> Line2d*;
};

}  // namespace deposition_simulator
