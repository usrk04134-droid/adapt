#pragma once

namespace tracking {

class SmoothMotion {
 public:
  /*high_movement_threashold: The amount of mm difference for the target position to be updated with 90% */
  explicit SmoothMotion(double high_movement_threshold);
  auto GetSmoothTarget(double current, double target) const -> double;

 private:
  auto GetRateOfChange(double current, double target) const -> double;
  double change_factor_;
};
}  // namespace tracking
