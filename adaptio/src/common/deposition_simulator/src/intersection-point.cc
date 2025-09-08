
#include "intersection-point.h"

#include "line2d.h"
#include "point2d.h"

namespace deposition_simulator {

IntersectionPoint::IntersectionPoint() : Point2d(0.0, 0.0), intersectedLine_(nullptr) {};

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
IntersectionPoint::IntersectionPoint(double x_coord, double y_coord, Line2d* line)
    : Point2d(x_coord, y_coord), intersectedLine_(line) {}

auto IntersectionPoint::GetIntersectedLine() const -> Line2d* { return this->intersectedLine_; }
}  // namespace deposition_simulator
