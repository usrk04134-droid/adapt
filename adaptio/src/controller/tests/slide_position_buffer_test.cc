#include "../slide_position_buffer.h"

#include <doctest/doctest.h>

#include <cstdint>

using controller::SlidePositionBufferImpl;

// NOLINTBEGIN(*-magic-numbers, *-optional-access)

TEST_SUITE("Slide Position Buffer") {
  TEST_CASE("Store") {
    SlidePositionBufferImpl buffer;
    buffer.StorePosition(1.0, 100);

    auto res = buffer.GetPosition(120);
    CHECK_EQ(res, 1.0);

    buffer.StorePosition(2.0, 120);

    res = buffer.GetPosition(90);
    CHECK_EQ(res, 1.0);

    res = buffer.GetPosition(130);
    CHECK_EQ(res, 2.0);

    res = buffer.GetPosition(109);
    CHECK_EQ(res, 1.45);

    res = buffer.GetPosition(111);
    CHECK_EQ(res, 1.55);
  }

  TEST_CASE("Max buffer") {
    SlidePositionBufferImpl buffer;

    for (uint64_t i = 0; i < buffer.Capacity(); i++) {
      buffer.StorePosition(static_cast<float>(i), i * 20);
    }

    auto res = buffer.GetPosition(0);
    CHECK_EQ(res, 0.0);

    // Buffer should be full. First entry swapped out
    buffer.StorePosition(10.0, 2000);

    res = buffer.GetPosition(0);
    CHECK_EQ(res, 1.0);
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access)
