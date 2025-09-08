#include "scanner/joint_tracking/joint_tracking.h"

#include <doctest/doctest.h>

#include <array>

#include "scanner/joint_tracking/joint_slice.h"

// NOLINTBEGIN(*-magic-numbers)
namespace scanner::joint_tracking {

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

    JointSlice slice(testPoints, common::joint_tracking::SliceConfidence::HIGH);

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

    JointSlice slice(testPoints, common::joint_tracking::SliceConfidence::HIGH);

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

    JointSlice slice(testPoints, common::joint_tracking::SliceConfidence::HIGH);

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

}  // namespace scanner::joint_tracking
// NOLINTEND(*-magic-numbers)
