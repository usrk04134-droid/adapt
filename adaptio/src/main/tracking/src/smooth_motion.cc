#include "tracking/src/smooth_motion.h"

#include <cmath>
const double RATE_AT_THRESHOLD = 0.9;
namespace tracking {

// Constructor to initialize the threshold for high movement
SmoothMotion::SmoothMotion(double high_movement_threshold)
    : change_factor_(-std::log(1 - RATE_AT_THRESHOLD) / high_movement_threshold) {}

// Function to get the smoothly updated target
auto SmoothMotion::GetSmoothTarget(double current, double target) const -> double {
  double rate_of_change = GetRateOfChange(current, target);
  return current + rate_of_change * (target - current);
}

// Function to compute the rate of change based on current and target
auto SmoothMotion::GetRateOfChange(double current, double target) const -> double {
  double distance = std::abs(target - current);
  return 1 - std::exp(-change_factor_ * distance);
}

}  // namespace tracking
