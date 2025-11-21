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
    CHECK_EQ(pb.Get(1.0, 1.0), 1);
  }

  TEST_CASE("two entries") {
    common::containers::RelativePositionBuffer<int> pb(10);

    pb.Store(1.0, 1);
    pb.Store(1.5, 2);

    CHECK_EQ(pb.Size(), 2);
    CHECK(!pb.Empty());

    CHECK_EQ(pb.Get(1.5, 0.3), 2);
    CHECK_EQ(pb.Get(1.5, 0.6), 1);
  }

  TEST_CASE("position ahead of stored reference clamps remaining distance") {
    common::containers::RelativePositionBuffer<int> pb(10);

    pb.Store(1.0, 1);
    pb.Store(1.5, 2);

    CHECK_FALSE(pb.Get(2.0, 0.3).has_value());
    CHECK_EQ(pb.Get(1.7, 0.6), 2);
    CHECK_EQ(pb.Get(1.4, 0.3), 2);
    CHECK_EQ(pb.Get(1.4, 0.8), 1);
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
}

// NOLINTEND(*-magic-numbers, misc-include-cleaner)
