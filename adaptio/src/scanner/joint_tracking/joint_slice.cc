#include "scanner/joint_tracking/joint_slice.h"

#include <cstddef>

namespace scanner::joint_tracking {

double JointSlice::InterpolateZ(double x) const {
  // Assuming points_ are sorted in ascending order of x
  if (x <= points_.front().x) {
    return points_.front().z;
  }

  if (x >= points_.back().x) {
    return points_.back().z;
  }

  // Interpolate between points_ with closest x values.
  for (std::size_t i = 1; i < points_.size(); ++i) {
    if (points_[i].x >= x) {
      const auto& leftPoint  = points_[i - 1];
      const auto& rightPoint = points_[i];

      double xDiff = rightPoint.x - leftPoint.x;
      // Avoid division by zero
      if (xDiff <= 0) {
        return leftPoint.z;
      }

      double t = (x - leftPoint.x) / xDiff;
      return leftPoint.z + t * (rightPoint.z - leftPoint.z);
    }
  }
  // This should never be reached for a valid slice.
  return points_.back().z;
}

}  // namespace scanner::joint_tracking
