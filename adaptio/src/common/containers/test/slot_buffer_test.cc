#include "common/containers/slot_buffer.h"

#include <doctest/doctest.h>

#include <numbers>
#include <optional>

// NOLINTBEGIN(*-magic-numbers, misc-include-cleaner)

TEST_SUITE("SlotBuffer") {
  TEST_CASE("IndexMappingSingleValue") {
    common::containers::SlotBuffer<double> buf(360, 360.0);

    buf.Store(0.5, 10.0);

    CHECK(std::fabs(buf.Get(0.1).value().second - 10.0) < std::numeric_limits<double>::epsilon());
  }

  TEST_CASE("IndexMappingSingleValueOverwrite") {
    common::containers::SlotBuffer<double> buf(360, 360.0);

    buf.Store(0.5, 10.0);
    CHECK(std::fabs(buf.Get(0.4).value().second - 10.0) < std::numeric_limits<double>::epsilon());

    buf.Store(0.1, 20.0);
    CHECK(std::fabs(buf.Get(0.4).value().second - 10.0) < std::numeric_limits<double>::epsilon());

    buf.Store(0.8, 34.0);
    CHECK(std::fabs(buf.Get(0.4).value().second - 10.0) < std::numeric_limits<double>::epsilon());
  }

  TEST_CASE("IndexMappingMultipleValue") {
    common::containers::SlotBuffer<double> buf(360, 360.0);

    buf.Store(0.5, 10.0);
    buf.Store(2.5, 11.0);
    buf.Store(55.5, 12.0);
    buf.Store(359.9, 13.0);

    CHECK(std::fabs(buf.Get(0.1).value().second - 10.0) < std::numeric_limits<double>::epsilon());
    CHECK_FALSE(buf.Get(1.1).has_value());
    CHECK(std::fabs(buf.Get(2.1).value().second - 11.0) < std::numeric_limits<double>::epsilon());
    CHECK(std::fabs(buf.Get(55.1).value().second - 12.0) < std::numeric_limits<double>::epsilon());
    CHECK(std::fabs(buf.Get(359.1).value().second - 13.0) < std::numeric_limits<double>::epsilon());
  }

  TEST_CASE("WrapAround") {
    common::containers::SlotBuffer<double> buf(360, 360.0);

    buf.Store(360.5, 10.0);

    CHECK(std::fabs(buf.Get(0.1).value().second - 10.0) < std::numeric_limits<double>::epsilon());
  }

  TEST_CASE("Clear") {
    common::containers::SlotBuffer<double> buf(360, 360.0);

    buf.Store(0.5, 10.0);
    buf.Store(1.5, 11.0);
    buf.Store(55.5, 12.0);
    buf.Store(359.9, 13.0);

    CHECK(std::fabs(buf.Get(0.1).value().second - 10.0) < std::numeric_limits<double>::epsilon());
    CHECK(std::fabs(buf.Get(1.1).value().second - 11.0) < std::numeric_limits<double>::epsilon());
    CHECK(std::fabs(buf.Get(55.1).value().second - 12.0) < std::numeric_limits<double>::epsilon());
    CHECK(std::fabs(buf.Get(359.1).value().second - 13.0) < std::numeric_limits<double>::epsilon());

    buf.Clear();

    for (int i = 0; i < 360; i++) {
      CHECK_FALSE(buf.Get(i).has_value());
    }
  }

  TEST_CASE("Empty") {
    common::containers::SlotBuffer<double> buf(360, 360.0);

    CHECK(buf.Empty());

    buf.Store(55.5, 12.0);

    CHECK_FALSE(buf.Empty());
  }

  TEST_CASE("Filled") {
    common::containers::SlotBuffer<double> buf(10, 360.0);

    CHECK_FALSE(buf.Filled());

    for (int i = 0; i < 10; i++) {
      buf.Store(i * 36.0, 10);
    }

    CHECK(buf.Filled());
  }

  TEST_CASE("CalculateSlot") {
    common::containers::SlotBuffer<double> buf(100, 2 * std::numbers::pi);

    // Test slot calculation
    auto slot0     = buf.CalculateSlot(0.0);
    auto slot1     = buf.CalculateSlot((2 * std::numbers::pi) / 100);  // One slot size
    auto slot_wrap = buf.CalculateSlot(2 * std::numbers::pi);          // Should wrap to 0

    CHECK_EQ(slot0, 0);
    CHECK_EQ(slot1, 1);
    CHECK_EQ(slot_wrap, 0);  // Should wrap around

    auto slot_mid = buf.CalculateSlot(std::numbers::pi);  // Half way around
    CHECK_EQ(slot_mid, 49);
  }
}
// NOLINTEND(*-magic-numbers, misc-include-cleaner)
