#include "scanner/joint_tracking/joint_slice.h"

#include <doctest/doctest.h>

#include <array>
#include <cstddef>

// NOLINTBEGIN(*-magic-numbers)
namespace scanner::joint_tracking {

TEST_SUITE("JointSlice") {
  TEST_CASE("Test Corner point functions") {
    std::array<Coord, 7> testPoints = {
        {{0.0, 1.0}, {1.0, 0.0}, {2.0, -1.0}, {3.0, -2.0}, {4.0, -1.5}, {5.0, -1}, {6.0, 1.0}}
    };

    JointSlice slice(testPoints, common::joint_tracking::SliceConfidence::HIGH);

    CHECK_EQ(slice.GetTopLeftCorner().x, 0.0);
    CHECK_EQ(slice.GetTopLeftCorner().z, 1.0);

    CHECK_EQ(slice.GetTopRightCorner().x, 6.0);
    CHECK_EQ(slice.GetTopRightCorner().z, 1.0);
  }

  TEST_CASE("Test InterpolateZ function") {
    std::array<Coord, 7> testPoints = {
        {{0.0, 1.0}, {1.0, 0.0}, {2.0, -1.0}, {3.0, -2.0}, {4.0, -1.5}, {5.0, -1}, {6.0, 1.0}}
    };

    JointSlice slice(testPoints, common::joint_tracking::SliceConfidence::HIGH);

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

}  // namespace scanner::joint_tracking
// NOLINTEND(*-magic-numbers)
