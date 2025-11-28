#include "common/containers/position_buffer.h"

#include <doctest/doctest.h>

#include <optional>

// NOLINTBEGIN(*-magic-numbers, misc-include-cleaner)

TEST_SUITE("PositionBuffer") {
  TEST_CASE("Empty") {
    common::containers::PositionBuffer<int> buf(10);

    CHECK_EQ(buf.Size(), 0);
    CHECK(buf.Empty());

    CHECK_EQ(buf.Get(1.1), std::nullopt);
  }

  TEST_CASE("SingleEntry") {
    common::containers::PositionBuffer<int> buf(10);

    buf.Store(1.1, 5);

    CHECK_EQ(buf.Size(), 1);
    CHECK(!buf.Empty());

    CHECK_EQ(buf.Get(1.1).value(), 5);
  }

  TEST_CASE("MultipleEntries") {
    common::containers::PositionBuffer<int> buf(10);

    buf.Store(1.1, 5);
    buf.Store(1.1, 6);
    buf.Store(1.2, 7);

    CHECK_EQ(buf.Size(), 3);
    CHECK(!buf.Empty());

    CHECK_EQ(buf.Get(1.0).value(), 5);
    CHECK_EQ(buf.Get(1.1).value(), 5);
    CHECK_EQ(buf.Get(1.2).value(), 7);
    CHECK_EQ(buf.Get(1.3).value(), 7);
    CHECK_EQ(buf.Get(1.3).value(), 7);

    buf.Store(9.9, 11);

    CHECK_EQ(buf.Get(9.9).value(), 11);
    CHECK_EQ(buf.Get(10.1), std::nullopt);

    /* wrapped */
    CHECK_EQ(buf.Get(0.1).value(), 11);
    CHECK_EQ(buf.Get(1.0).value(), 5);
  }

  TEST_CASE("Clear") {
    common::containers::PositionBuffer<int> buf(10);

    buf.Clear();
    CHECK_EQ(buf.Size(), 0);
    CHECK(buf.Empty());

    buf.Store(1.1, 5);
    CHECK_EQ(buf.Size(), 1);
    CHECK(!buf.Empty());

    buf.Clear();

    CHECK_EQ(buf.Size(), 0);
    CHECK(buf.Empty());

    buf.Store(1.1, 5);
    CHECK_EQ(buf.Size(), 1);
    CHECK(!buf.Empty());
  }

  TEST_CASE("OutsideBounds") {
    common::containers::PositionBuffer<int> buf(10);

    buf.Store(10.1, 5);

    CHECK_EQ(buf.Size(), 0);
    CHECK(buf.Empty());

    CHECK_EQ(buf.Get(1.1), std::nullopt);

    buf.Store(9.9, 11);
    CHECK_EQ(buf.Get(9.9).value(), 11);
    CHECK_EQ(buf.Get(10.1), std::nullopt);
  }

  TEST_CASE("MultipleEntries Optional") {
    common::containers::PositionBuffer<int> buf(10);
    std::optional<common::containers::PositionBuffer<int>> opt_buf;

    buf.Store(1.1, 5);
    buf.Store(1.1, 6);
    buf.Store(1.2, 7);

    CHECK_EQ(buf.Size(), 3);
    CHECK(!buf.Empty());

    CHECK_EQ(buf.Get(1.0).value(), 5);
    CHECK_EQ(buf.Get(1.1).value(), 5);
    CHECK_EQ(buf.Get(1.2).value(), 7);
    CHECK_EQ(buf.Get(1.3).value(), 7);
    CHECK_EQ(buf.Get(1.3).value(), 7);

    opt_buf = buf;
    CHECK(opt_buf.has_value());
    CHECK_EQ(opt_buf.value().Get(1.0).value(), 5);
    CHECK_EQ(opt_buf.value().Get(1.1).value(), 5);
    CHECK_EQ(opt_buf.value().Get(1.2).value(), 7);
    CHECK_EQ(opt_buf.value().Get(1.3).value(), 7);
    CHECK_EQ(opt_buf.value().Get(1.3).value(), 7);
  }

  TEST_CASE("WrapAround") {
    common::containers::PositionBuffer<int> buf(2 * std::numbers::pi);

    auto const offset  = 4;
    auto const samples = 20;

    for (int i = 0; i < samples; ++i) {
      double const indx = (i + offset) % samples;
      auto const pos    = (indx / samples) * 2 * std::numbers::pi;
      buf.Store(pos, i);
    }

    for (int i = 0; i < samples; ++i) {
      double const pos = (static_cast<double>(i) / samples) * 2 * std::numbers::pi;
      CHECK(buf.Get(pos).value() == (samples + i - offset) % samples);
    }
  }

  TEST_CASE("NO_WRAP") {
    common::containers::PositionBuffer<int> buf(1.0, common::containers::WrapMode::NO_WRAP);

    buf.Store(0.0, 1);
    buf.Store(0.1, 2);
    buf.Store(0.2, 3);

    CHECK_EQ(buf.Get(0.9).value(), 3);
  }

  TEST_CASE("WRAP") {
    common::containers::PositionBuffer<int> buf(1.0, common::containers::WrapMode::WRAP);

    buf.Store(0.0, 1);
    buf.Store(0.1, 2);
    buf.Store(0.2, 3);

    CHECK_EQ(buf.Get(0.9).value(), 1);
  }
}
// NOLINTEND(*-magic-numbers, misc-include-cleaner)
