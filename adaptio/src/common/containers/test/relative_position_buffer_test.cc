#include "common/containers/relative_position_buffer.h"

#include <doctest/doctest.h>

#include <optional>

// NOLINTBEGIN(*-magic-numbers)

TEST_SUITE("RelativePositionBuffer") {
  TEST_CASE("returns nullopt when empty") {
    common::containers::RelativePositionBuffer<int> pb(10);

    CHECK_EQ(pb.Size(), 0);
    CHECK(pb.Empty());
    CHECK_EQ(pb.Get(1.1, 0.5), std::nullopt);
  }

  TEST_CASE("stores single entry and deduplicates by position") {
    common::containers::RelativePositionBuffer<int> pb(10);

    pb.Store(1.0, 1);

    CHECK_EQ(pb.Size(), 1);

    pb.Store(1.0, 2);
    CHECK_EQ(pb.Size(), 1);

    CHECK(!pb.Empty());

    CHECK_EQ(pb.Get(10.0, 9.0), 1);
    CHECK_EQ(pb.Get(1.0, 0.0), 1);
  }

  TEST_CASE("returns closest position for requested distance") {
    common::containers::RelativePositionBuffer<int> pb(10);

    pb.Store(3.0, 3);
    pb.Store(7.0, 7);
    pb.Store(9.0, 9);
    pb.Store(12.0, 12);

    CHECK_EQ(pb.Size(), 4);
    CHECK(!pb.Empty());

    CHECK_EQ(pb.Get(18.0, 5.0).value(), 12);
    CHECK_EQ(pb.Get(18.0, 8.0).value(), 9);
    CHECK_EQ(pb.Get(18.0, 15.0).value(), 3);
  }

  TEST_CASE("prefers newest entry when equidistant") {
    common::containers::RelativePositionBuffer<int> pb(10);

    pb.Store(10.0, 1);
    pb.Store(20.0, 2);

    CHECK_EQ(pb.Get(25.0, 10.0).value(), 2);
  }

  TEST_CASE("handles negative positions") {
    common::containers::RelativePositionBuffer<int> pb(10);

    pb.Store(-1.5, 1);
    pb.Store(-1.0, 2);

    CHECK_EQ(pb.Size(), 2);
    CHECK(!pb.Empty());

    CHECK_EQ(pb.Get(-0.5, 0.5).value(), 2);
    CHECK_EQ(pb.Get(-0.5, 1.5).value(), 1);
  }
}

// NOLINTEND(*-magic-numbers)
