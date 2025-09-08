#include "core/joint_tracking/joint_tracking.h"

#include "core/joint_tracking/joint_slice.h"

using core::joint_tracking::Coord;
using core::joint_tracking::JointSlice;
using core::joint_tracking::JointTracking;

Coord JointTracking::CalculateRelativePosition(const Coord& currentPosition, const JointSlice& jointSlice) {
  auto targetPosition = CalculateTargetPosition_(currentPosition, jointSlice);
  return Coord{targetPosition.x - currentPosition.x, targetPosition.z - currentPosition.z};
}

Coord JointTracking::CalculateTargetPosition_(const Coord& currentPosition, const JointSlice& jointSlice) {
  double xTarget, zTarget;
  if (trackHorizontal_) {
    xTarget = GetHorizontalPosition_(jointSlice);
  } else {
    xTarget = currentPosition.x;
  }

  if (trackVertical_) {
    zTarget = jointSlice.InterpolateZ(xTarget) + stickout_;
  } else {
    zTarget = currentPosition.z;
  }
  return Coord{xTarget, zTarget};
}

double JointTracking::GetHorizontalPosition_(const JointSlice& jointSlice) {
  auto leftX   = jointSlice.GetBottomLeftCorner().x;
  auto rightX  = jointSlice.GetBottomRightCorner().x;
  leftX       += wallOffset_;
  rightX      -= wallOffset_;

  if (leftX >= rightX) {
    return (leftX + rightX) / 2.0;
  }

  auto placement = horizontalPlacement_;

  if (!trackFromLeft_) {
    placement = 1.0 - placement;
  }
  return leftX + placement * (rightX - leftX);
}

#ifndef DOCTEST_CONFIG_DISABLE
// NOLINTBEGIN(*-magic-numbers)

#include <doctest/doctest.h>

#include <array>

TEST_SUITE("JointTracking") {
  TEST_CASE("Horizontal only") {
    double stickout      = 25.0;
    double wallOffset    = 4.0;
    double placement     = 0.0;
    bool trackHorizontal = true;
    bool trackVertical   = false;
    bool trackFromLeft   = true;
    JointTracking jointTracking(wallOffset, stickout, placement, trackHorizontal, trackVertical, trackFromLeft);

    std::array<Coord, 7> testPoints = {
        {{0.0, 10.0}, {10.0, 0.0}, {20.0, -10.0}, {30.0, -20.0}, {40.0, -15.0}, {50.0, -10.0}, {60.0, 10.0}}
    };

    JointSlice slice(testPoints, core::joint_tracking::SliceConfidence::HIGH);

    Coord currentPosition{15, 2.0};
    // Track left side
    auto relativePos = jointTracking.CalculateRelativePosition(currentPosition, slice);

    CHECK_EQ(relativePos.x, (10.0 + wallOffset) - currentPosition.x);
    CHECK_EQ(relativePos.z, 0);

    // Track middle
    jointTracking.SetHorizontalPlacement(0.5);
    relativePos = jointTracking.CalculateRelativePosition(currentPosition, slice);

    CHECK_EQ(relativePos.x, (30.0) - currentPosition.x);
    CHECK_EQ(relativePos.z, 0);

    // Track right side
    jointTracking.SetHorizontalPlacement(1.0);
    relativePos = jointTracking.CalculateRelativePosition(currentPosition, slice);

    CHECK_EQ(relativePos.x, (50.0 - wallOffset) - currentPosition.x);
    CHECK_EQ(relativePos.z, 0);

    // Track left side with rightside tracking
    jointTracking.SetTrackingSide(false);

    relativePos = jointTracking.CalculateRelativePosition(currentPosition, slice);

    CHECK_EQ(relativePos.x, (10.0 + wallOffset) - currentPosition.x);
    CHECK_EQ(relativePos.z, 0);
  }
  TEST_CASE("Vertical only") {
    double stickout      = 25.0;
    double wallOffset    = 4.0;
    double placement     = 0.0;
    bool trackHorizontal = false;
    bool trackVertical   = true;
    bool trackFromLeft   = true;
    JointTracking jointTracking(wallOffset, stickout, placement, trackHorizontal, trackVertical, trackFromLeft);

    std::array<Coord, 7> testPoints = {
        {{0.0, 10.0}, {10.0, 0.0}, {20.0, -10.0}, {30.0, -20.0}, {40.0, -15.0}, {50.0, -10.0}, {60.0, 10.0}}
    };

    JointSlice slice(testPoints, core::joint_tracking::SliceConfidence::HIGH);

    Coord currentPosition{10.0, 2.0};

    auto relativePos = jointTracking.CalculateRelativePosition(currentPosition, slice);

    CHECK_EQ(relativePos.x, 0.0);
    CHECK_EQ(relativePos.z, stickout + 0.0 - currentPosition.z);

    currentPosition.x = 15.0;

    relativePos = jointTracking.CalculateRelativePosition(currentPosition, slice);

    CHECK_EQ(relativePos.x, 0.0);
    CHECK_EQ(relativePos.z, stickout + -5.0 - currentPosition.z);

    currentPosition.x = 80.0;

    relativePos = jointTracking.CalculateRelativePosition(currentPosition, slice);

    CHECK_EQ(relativePos.x, 0.0);
    CHECK_EQ(relativePos.z, stickout + 10.0 - currentPosition.z);
  }
  TEST_CASE("Full tracking") {
    double stickout      = 25.0;
    double wallOffset    = 5.0;
    double placement     = 0.0;
    bool trackHorizontal = true;
    bool trackVertical   = true;
    bool trackFromLeft   = true;
    JointTracking jointTracking(wallOffset, stickout, placement, trackHorizontal, trackVertical, trackFromLeft);

    std::array<Coord, 7> testPoints = {
        {{0.0, 10.0}, {10.0, 0.0}, {20.0, -10.0}, {30.0, -20.0}, {40.0, -15.0}, {50.0, -10.0}, {60.0, 10.0}}
    };

    JointSlice slice(testPoints, core::joint_tracking::SliceConfidence::HIGH);

    Coord currentPosition{10.0, 2.0};

    auto relativePos = jointTracking.CalculateRelativePosition(currentPosition, slice);

    CHECK_EQ(relativePos.x, wallOffset + 10.0 - currentPosition.x);
    CHECK_EQ(relativePos.z, stickout - 5.0 - currentPosition.z);

    currentPosition.x += relativePos.x;
    currentPosition.z += relativePos.z;

    relativePos = jointTracking.CalculateRelativePosition(currentPosition, slice);

    CHECK_EQ(relativePos.x, 0);
    CHECK_EQ(relativePos.z, 0);
  }
}

// NOLINTEND(*-magic-numbers)
#endif
