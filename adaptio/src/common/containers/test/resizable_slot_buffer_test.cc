#include "common/containers/resizable_slot_buffer.h"

#include <doctest/doctest.h>

#include <optional>

// NOLINTBEGIN(*-magic-numbers, misc-include-cleaner)

TEST_SUITE("ResizableSlotBuffer") {
  TEST_CASE("ResizeWithValues") {
    common::containers::ResizableSlotBuffer<double> buf(360, 360.0);

    buf.Store(0.5, 10.0);
    buf.Store(359.95, 20.0);

    CHECK(buf.Get(0.45).has_value());
    CHECK(buf.Get(0.55).has_value());
    CHECK(buf.Get(0.65).has_value());

    buf.Resize(3600);

    CHECK_FALSE(buf.Get(0.45).has_value());
    CHECK(buf.Get(0.55).has_value());
    CHECK_FALSE(buf.Get(0.65).has_value());

    CHECK(std::fabs(buf.Get(0.5).value().second - 10.0) < std::numeric_limits<double>::epsilon());
    CHECK_EQ(buf.Get(0.15), std::nullopt);

    CHECK(buf.Get(359.95).has_value());
    CHECK(std::fabs(buf.Get(359.95).value().second - 20.0) < std::numeric_limits<double>::epsilon());
    CHECK_EQ(buf.Get(359.85), std::nullopt);
  }
}
// NOLINTEND(*-magic-numbers, misc-include-cleaner)
