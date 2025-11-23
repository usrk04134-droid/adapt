#include "common/containers/relative_position_buffer.h"

#include <doctest/doctest.h>

#include <optional>

// NOLINTBEGIN(*-magic-numbers, misc-include-cleaner)

TEST_SUITE("RelativePositionBuffer") {
  TEST_CASE("Test no data") {
    common::containers::RelativePositionBuffer<int> pb(10);

    CHECK_EQ(pb.Size(), 0);
    CHECK(pb.Empty());
    CHECK_EQ(pb.Get(1.1, 0.5), std::nullopt);
  }

  TEST_CASE("one entry") {
    common::containers::RelativePositionBuffer<int> pb(10);

    pb.Store(1.0, 1);

    CHECK_EQ(pb.Size(), 1);

    pb.Store(1.0, 2);  // Same position, should update or ignore? Implementation ignores if same position at front.
    // "if (data_.front().position != position)"
    // Wait, Store implementation:
    // if (data_.empty() || data_.front().position != position) { push... }
    // So it ignores if same position.
    CHECK_EQ(pb.Size(), 1);

    CHECK(!pb.Empty());

    CHECK_EQ(pb.Get(1.0, 0.0).value(), 1);
    CHECK_EQ(pb.Get(2.0, 1.0).value(), 1);
    CHECK_EQ(pb.Get(10.0, 5.0).value(), 1);  // target 5, closest 1
  }

  TEST_CASE("multiple entries") {
    common::containers::RelativePositionBuffer<int> pb(10);

    // positions: 3, 7, 9, 12
    pb.Store(3.0, 3);
    pb.Store(7.0, 7);
    pb.Store(9.0, 9);
    pb.Store(12.0, 12);

    CHECK_EQ(pb.Size(), 4);

    // Get(18, 5) -> target 13. Closest 12.
    CHECK_EQ(pb.Get(18.0, 5.0).value(), 12);

    // Get(18, 8) -> target 10. Closest 9.
    // |12-10|=2, |9-10|=1.
    CHECK_EQ(pb.Get(18.0, 8.0).value(), 9);

    // Get(18, 10) -> target 8. Closest 7 or 9.
    // |9-8|=1, |7-8|=1.
    // Implementation picks older one (7) because of equality update.
    CHECK_EQ(pb.Get(18.0, 10.0).value(), 7);

    // Get(18, 12) -> target 6. Closest 7.
    CHECK_EQ(pb.Get(18.0, 12.0).value(), 7);

    // Get(18, 16) -> target 2. Closest 3.
    CHECK_EQ(pb.Get(18.0, 16.0).value(), 3);
  }

  TEST_CASE("Negative values") {
    common::containers::RelativePositionBuffer<int> pb(10);

    pb.Store(-10.0, 1);
    pb.Store(-5.0, 2);

    CHECK_EQ(pb.Size(), 2);

    // Get(-2, 2) -> target -4. Closest -5.
    CHECK_EQ(pb.Get(-2.0, 2.0).value(), 2);

    // Get(-2, 9) -> target -11. Closest -10.
    CHECK_EQ(pb.Get(-2.0, 9.0).value(), 1);
  }
}

// NOLINTEND(*-magic-numbers, misc-include-cleaner)
