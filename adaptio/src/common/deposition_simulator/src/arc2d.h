#pragma once

#include <vector>

#include "circle2d.h"
#include "line2d.h"
namespace deposition_simulator {

class Arc2d {
 private:
  double startAngle_;
  double endAngle_;
  bool counter_clock_wise_;
  Circle2d circle_;

  auto ComputeArcAngle() const -> double;

 public:
  Arc2d();
  Arc2d(const Circle2d& circle, double start_angle, double end_angle, bool ccw);
  auto Discretize(int nbr_lines) const -> std::vector<Line2d>;
};

}  // namespace deposition_simulator
