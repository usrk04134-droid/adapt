#include "common/containers/relative_position_buffer.h"

#include <doctest/doctest.h>

// NOLINTBEGIN(*-magic-numbers, misc-include-cleaner)

TEST_SUITE("RelativePositionBuffer") {
  TEST_CASE("Test no data") {
    common::containers::RelativePositionBuffer<int> pb(10);

    CHECK_EQ(pb.Size(), 0);
    CHECK(pb.Empty());
    CHECK_EQ(pb.Get(0.0, 1.1), std::nullopt);
  }

  TEST_CASE("one entry") {
    common::containers::RelativePositionBuffer<int> pb(10);

    pb.Store(1.0, 1);
    CHECK_EQ(pb.Size(), 1);

    pb.Store(1.0, 2);
    CHECK_EQ(pb.Size(), 1);
    CHECK(!pb.Empty());

    CHECK_EQ(pb.Get(1.0, 0.0), 1);
    CHECK_EQ(pb.Get(1.0, 0.5), std::nullopt);
  }

  TEST_CASE("two entries") {
    common::containers::RelativePositionBuffer<int> pb(10);

    pb.Store(1.0, 1);
    pb.Store(1.5, 2);

    CHECK_EQ(pb.Size(), 2);
    CHECK(!pb.Empty());

    CHECK_EQ(pb.Get(1.5, 0.3), 2);
    CHECK_EQ(pb.Get(1.5, 0.6), 1);
    CHECK_EQ(pb.Get(1.5, 1.0), std::nullopt);
  }

  TEST_CASE("insufficient history returns nullopt") {
    common::containers::RelativePositionBuffer<int> pb(10);

    pb.Store(1.0, 1);
    pb.Store(1.5, 2);

    CHECK_FALSE(pb.Get(2.0, 0.3).has_value());
  }

  TEST_CASE("Negative values") {
    common::containers::RelativePositionBuffer<int> pb(10);

    pb.Store(-1.5, 1);
    pb.Store(-1.0, 2);

    CHECK_EQ(pb.Size(), 2);
    CHECK(!pb.Empty());

    CHECK_EQ(pb.Get(-1.0, 0.3), 2);
    CHECK_EQ(pb.Get(-1.0, 0.6), 1);
  }

  TEST_CASE("returns closest available historical position") {
    common::containers::RelativePositionBuffer<int> pb(10);

    pb.Store(3.0, 3);
    pb.Store(7.0, 7);
    pb.Store(9.0, 9);
    pb.Store(12.0, 12);

    CHECK_EQ(pb.Get(18.0, 5.0), 12);
    CHECK_EQ(pb.Get(18.0, 8.0), 9);
    CHECK_EQ(pb.Get(18.0, 15.0), 3);
    CHECK_FALSE(pb.Get(18.0, 20.0).has_value());
  }
}

// NOLINTEND(*-magic-numbers, misc-include-cleaner)
