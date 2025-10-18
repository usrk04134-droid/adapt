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
    CHECK_EQ(pb.Get(1.1), std::nullopt);
  }

  TEST_CASE("one entry") {
    common::containers::RelativePositionBuffer<int> pb(10);

    pb.Store(1.0, 1);

    CHECK_EQ(pb.Size(), 1);

    pb.Store(1.0, 2);
    CHECK_EQ(pb.Size(), 1);

    CHECK(!pb.Empty());

    CHECK_EQ(pb.Get(0.0), 1);
    CHECK_EQ(pb.Get(1.0), 1);
  }

  TEST_CASE("two entries") {
    common::containers::RelativePositionBuffer<int> pb(10);

    pb.Store(1.0, 1);
    pb.Store(1.5, 2);

    CHECK_EQ(pb.Size(), 2);
    CHECK(!pb.Empty());

    CHECK_EQ(pb.Get(0.3), 2);
    CHECK_EQ(pb.Get(0.6), 1);
  }

  TEST_CASE("buffer wrap") {
    common::containers::RelativePositionBuffer<int> pb(10, 2 * std::numbers::pi);

    pb.Store(1.0, 1);
    pb.Store(2.0, 2);
    pb.Store(3.0, 3);
    pb.Store(4.0, 4);
    pb.Store(5.0, 5);
    pb.Store(6.0, 6);
    pb.Store(1.0, 7);
    pb.Store(2.0, 8);

    CHECK_EQ(pb.Get(0.5).value(), 8);
    CHECK_EQ(pb.Get(1.5).value(), 7);
    CHECK_EQ(pb.Get(2.0).value(), 7);
    CHECK_EQ(pb.Get(2.5).value(), 6);
  }
  TEST_CASE("Negative values") {
    common::containers::RelativePositionBuffer<int> pb(10);

    pb.Store(-1.5, 1);
    pb.Store(-1.0, 2);

    CHECK_EQ(pb.Size(), 2);
    CHECK(!pb.Empty());

    CHECK_EQ(pb.Get(0.3), 2);
    CHECK_EQ(pb.Get(0.6), 1);
  }
}

// NOLINTEND(*-magic-numbers, misc-include-cleaner)
