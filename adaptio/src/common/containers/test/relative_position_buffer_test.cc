#include "common/containers/relative_position_buffer.h"

#include <doctest/doctest.h>

#include <numbers>
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

    pb.Store(1.0, 2);
    CHECK_EQ(pb.Size(), 1);

    CHECK(!pb.Empty());

    CHECK_EQ(pb.Get(10.0, 9.0), 1);
    CHECK_EQ(pb.Get(1.0, 0.0), 1);
  }

  TEST_CASE("two entries") {
    common::containers::RelativePositionBuffer<int> pb(10);

    pb.Store(5.0, 5);
    pb.Store(10.0, 10);

    CHECK_EQ(pb.Get(3.0, 0.0).value(), 5);
  }

  TEST_CASE("multiple entries") {
    common::containers::RelativePositionBuffer<int> pb(10);

    pb.Store(3.0, 1);
    pb.Store(7.0, 2);
    pb.Store(9.0, 3);
    pb.Store(11.0, 4);

    CHECK_EQ(pb.Size(), 4);
    CHECK(!pb.Empty());

    CHECK_EQ(pb.Get(18.0, 5.0), 4);
    CHECK_EQ(pb.Get(18.0, 8.0), 4);
    CHECK_EQ(pb.Get(15.0, 15.0), 1);
    CHECK_EQ(pb.Get(18.0, 10.0), 3);
    CHECK_EQ(pb.Get(17.0, 11.0), 2);
  }

  TEST_CASE("Negative values") {
    common::containers::RelativePositionBuffer<int> pb(10);

    pb.Store(-1.5, 1);
    pb.Store(-1.0, 2);

    CHECK_EQ(pb.Size(), 2);
    CHECK(!pb.Empty());

    CHECK_EQ(pb.Get(-0.5, 0.5), 2);
    CHECK_EQ(pb.Get(-0.5, 1.5), 1);
  }
}

// NOLINTEND(*-magic-numbers, misc-include-cleaner)
