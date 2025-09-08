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

    auto data = pb.Get(1.1, 1.3);
    CHECK_EQ(data.Size(), 0);
    CHECK(data.Empty());

    for (auto it : data) {
      /* should not be called */
      FAIL(it);
    }

    CHECK_EQ(pb.Get(1.1), std::nullopt);
  }

  TEST_CASE("one entry") {
    common::containers::RelativePositionBuffer<int> pb(10);

    pb.Store(1.0, 1);

    CHECK_EQ(pb.Size(), 1);
    CHECK(!pb.Empty());

    CHECK_EQ(pb.Get(0.0, common::containers::Round::MIN), 1);
    CHECK_EQ(pb.Get(0.0, common::containers::Round::CLOSEST), 1);
    CHECK_EQ(pb.Get(1.0, common::containers::Round::MIN), 1);
    CHECK_EQ(pb.Get(1.0, common::containers::Round::CLOSEST), 1);

    CHECK_EQ(pb.Get(0.00, 0.45).Size(), 1);
  }

  TEST_CASE("two entries") {
    common::containers::RelativePositionBuffer<int> pb(10);

    pb.Store(1.0, 1);
    pb.Store(1.5, 2);

    CHECK_EQ(pb.Size(), 2);
    CHECK(!pb.Empty());

    CHECK_EQ(pb.Get(0., common::containers::Round::CLOSEST), 2);
    CHECK_EQ(pb.Get(0.45, common::containers::Round::CLOSEST), 1);

    CHECK_EQ(pb.Get(0.3, common::containers::Round::MIN), 2);

    CHECK_EQ(pb.Get(0.6, common::containers::Round::CLOSEST), 1);
    CHECK_EQ(pb.Get(0.6, common::containers::Round::MIN), 1);
  }

  TEST_CASE("iterator") {
    common::containers::RelativePositionBuffer<int> pb(10);

    pb.Store(1.0, 1);
    pb.Store(1.1, 2);
    pb.Store(1.2, 3);
    pb.Store(1.3, 4);
    pb.Store(1.4, 5);
    pb.Store(1.5, 6);

    CHECK_EQ(pb.Size(), 6);
    CHECK(!pb.Empty());

    auto data = pb.Get(0.0, 0.5);
    CHECK_EQ(data.Size(), 6);
    CHECK(!data.Empty());

    CHECK_EQ(pb.Get(0.00, 0.45).Size(), 5);
    CHECK_EQ(pb.Get(0.00, 0.41).Size(), 5);
    CHECK_EQ(pb.Get(0.00, 0.39).Size(), 4);
    CHECK_EQ(pb.Get(0.00, 0.11).Size(), 2);
    CHECK_EQ(pb.Get(0.00, 0.05).Size(), 1);

    CHECK_EQ(pb.Get(0.09, 0.15).Size(), 1);
    CHECK_EQ(pb.Get(0.11, 0.50).Size(), 4);
    CHECK_EQ(pb.Get(0.41, 0.50).Size(), 1);
    CHECK_EQ(pb.Get(0.51, 5.50).Size(), 0);
  }

  TEST_CASE("buffer wrap") {
    auto const size    = 10;
    auto const entries = 50;
    common::containers::RelativePositionBuffer<int> pb(size);

    auto step = 2 * std::numbers::pi / (size + 1.0);
    auto pos  = 0.;
    for (int i = 0; i <= entries; ++i) {
      pb.Store(pos, i);
      pos += step;
    }

    auto data = pb.Get(0.00, 2 * std::numbers::pi);
    CHECK_EQ(data.Size(), size);

    auto expect = entries;
    for (auto it : data) {
      CHECK_EQ(expect--, it.data);
    }
  }

  TEST_CASE("Test get large data") {
    auto const size = 220000;
    common::containers::RelativePositionBuffer<int> pb(size);

    auto step = 2 * std::numbers::pi / 100000.0;
    // Store 100000 values for whole turn
    for (double i = 0.0; i < 2 * std::numbers::pi; i += step) {
      pb.Store(i, 1);
    }
    // Store 100000 values for a new turn
    for (double i = 0.0; i < 2 * std::numbers::pi; i += step) {
      pb.Store(i, 2);
    }

    // Get 80% of the values for the latest turn
    auto data = pb.Get(0.1 * 2 * std::numbers::pi, 0.9 * 2 * std::numbers::pi);
    CHECK(data.Size() == 80000);
    CHECK(data.begin()->data == 2);
    auto it = data.end() - 1;
    CHECK(it->data == 2);

    // Get 80% of the values for the previous turn
    data = pb.Get(1.1 * 2 * std::numbers::pi, 1.9 * 2 * std::numbers::pi);

    CHECK(data.Size() == 80000);
    CHECK(data.begin()->data == 1);
    it = data.end() - 1;
    CHECK(it->data == 1);
  }
}

// NOLINTEND(*-magic-numbers, misc-include-cleaner)
