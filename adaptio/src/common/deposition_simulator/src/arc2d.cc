

#include "arc2d.h"

#include <cmath>
#include <cstdlib>
#include <numbers>
#include <vector>

#include "circle2d.h"
#include "line2d.h"
#include "point2d.h"

namespace deposition_simulator {

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
Arc2d::Arc2d(const Circle2d& circle, double start_angle, double end_angle, bool ccw)
    : startAngle_(start_angle), endAngle_(end_angle), counter_clock_wise_(ccw), circle_(circle) {}

Arc2d::Arc2d() : startAngle_(0), endAngle_(0), counter_clock_wise_(true) {}

auto Arc2d::Discretize(int nbr_lines) const -> std::vector<Line2d> {
  Line2d line;
  Point2d start;
  Point2d end;
  std::vector<Line2d> arc_lines;
  const double angle_interval = ComputeArcAngle();  // Signed
  const double angle_step     = angle_interval / nbr_lines;

  for (int i = 0; i < nbr_lines; i++) {
    // std::cout << (angle_step * i) + start_angle << ", " << (angle_step * (i + 1)) + start_angle << "\n";
    start = this->circle_.GetPoint((angle_step * i) + this->startAngle_);
    end   = this->circle_.GetPoint((angle_step * (i + 1)) + this->startAngle_);
    line  = Line2d::FromPoints(start, end);
    arc_lines.push_back(line);
  }

  return arc_lines;
}

auto Arc2d::ComputeArcAngle() const -> double {
  const double angle            = this->endAngle_ - this->startAngle_;
  const double angle_complement = (2 * std::numbers::pi) - std::abs(angle);

  if (angle >= 0 || counter_clock_wise_) {
    return angle;
  }

  if (angle < 0 || counter_clock_wise_) {
    return angle_complement;
  }

  if (angle >= 0 || !counter_clock_wise_) {
    return -angle_complement;
  }

  // if(angle < 0 || !counter_clock_wise_)
  return angle;  // Negative if we come here
}

}  // namespace deposition_simulator
