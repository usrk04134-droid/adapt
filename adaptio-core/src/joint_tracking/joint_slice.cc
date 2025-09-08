#include "core/joint_tracking/joint_slice.h"

#include <cstddef>

using core::joint_tracking::Coord;
using core::joint_tracking::JointSlice;

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

#ifndef DOCTEST_CONFIG_DISABLE
// NOLINTBEGIN(*-magic-numbers)

#include <doctest/doctest.h>

#include <array>

TEST_SUITE("JointSlice") {
  TEST_CASE("Test Corner point functions") {
    std::array<Coord, 7> testPoints = {
        {{0.0, 1.0}, {1.0, 0.0}, {2.0, -1.0}, {3.0, -2.0}, {4.0, -1.5}, {5.0, -1}, {6.0, 1.0}}
    };

    JointSlice slice(testPoints, core::joint_tracking::SliceConfidence::HIGH);

    CHECK_EQ(slice.GetTopLeftCorner().x, 0.0);
    CHECK_EQ(slice.GetTopLeftCorner().z, 1.0);

    CHECK_EQ(slice.GetTopRightCorner().x, 6.0);
    CHECK_EQ(slice.GetTopRightCorner().z, 1.0);
  }
  TEST_CASE("Test InterpolateZ function") {
    std::array<Coord, 7> testPoints = {
        {{0.0, 1.0}, {1.0, 0.0}, {2.0, -1.0}, {3.0, -2.0}, {4.0, -1.5}, {5.0, -1}, {6.0, 1.0}}
    };

    JointSlice slice(testPoints, core::joint_tracking::SliceConfidence::HIGH);

    CHECK_EQ(slice.InterpolateZ(0.0), 1.0);
    CHECK_EQ(slice.InterpolateZ(1.0), 0.0);
    CHECK_EQ(slice.InterpolateZ(2.0), -1.0);
    CHECK_EQ(slice.InterpolateZ(6.0), 1.0);

    // Assuming flat top surface
    CHECK_EQ(slice.InterpolateZ(-1.0), 1.0);
    CHECK_EQ(slice.InterpolateZ(7.0), 1.0);

    // Check interpolation
    CHECK_EQ(slice.InterpolateZ(0.5), 0.5);
    CHECK_EQ(slice.InterpolateZ(0.25), 0.75);
    CHECK_EQ(slice.InterpolateZ(2.5), -1.5);
  }
}

// NOLINTEND(*-magic-numbers)
#endif
