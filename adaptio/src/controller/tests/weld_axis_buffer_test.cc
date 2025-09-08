#include "../weld_axis_buffer.h"

#include <doctest/doctest.h>

#include <cstdint>

using controller::WeldAxisBufferImpl;

// NOLINTBEGIN(*-magic-numbers, *-optional-access)

TEST_SUITE("Weld Position Buffer") {
  TEST_CASE("Store") {
    WeldAxisBufferImpl buffer;
    buffer.StorePosition(1.0, 2.0, 100);
    auto res = buffer.GetPosition(120);

    REQUIRE(res.position == doctest::Approx(1.0));
    REQUIRE(res.velocity == doctest::Approx(2.0));
    CHECK_EQ(res.turn_counter, 0);

    buffer.StorePosition(2.0, 3.0, 120);

    res = buffer.GetPosition(109);
    REQUIRE(res.position == doctest::Approx(1.0));
    REQUIRE(res.velocity == doctest::Approx(2.0));
    CHECK_EQ(res.turn_counter, 0);

    res = buffer.GetPosition(111);
    REQUIRE(res.position == doctest::Approx(2.0));
    REQUIRE(res.velocity == doctest::Approx(3.0));
    CHECK_EQ(res.turn_counter, 0);
  }

  TEST_CASE("Max buffer") {
    WeldAxisBufferImpl buffer;

    for (uint64_t i = 0; i < buffer.Capacity(); i++) {
      buffer.StorePosition(static_cast<float>(i), static_cast<float>(2 * i), i * 20);
    }

    auto res = buffer.GetPosition(0);
    REQUIRE(res.position == doctest::Approx(0.0));
    REQUIRE(res.velocity == doctest::Approx(0.0));

    // Buffer should be full. First element swapped out
    buffer.StorePosition(10.0, 20.0, 2000);

    res = buffer.GetPosition(0);
    REQUIRE(res.position == doctest::Approx(1.0));
    REQUIRE(res.velocity == doctest::Approx(2.0));
    CHECK_EQ(res.turn_counter, 0);
  }

  TEST_CASE("Turn") {
    WeldAxisBufferImpl buffer;
    for (uint64_t i = 0; i < buffer.Capacity(); i++) {
      buffer.StorePosition(static_cast<float>(i), static_cast<float>(2 * i), i * 20);
    }

    auto res = buffer.GetPosition(1995);
    REQUIRE(res.position == doctest::Approx(99.0));
    REQUIRE(res.velocity == doctest::Approx(2 * 99.0));
    CHECK_EQ(res.turn_counter, 0);

    // Position is wrapped i.e. new turn
    buffer.StorePosition(1.0, 2.0, 2000);

    res = buffer.GetPosition(1995);
    REQUIRE(res.position == doctest::Approx(1.0));
    REQUIRE(res.velocity == doctest::Approx(2.0));
    CHECK_EQ(res.turn_counter, 1);
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access)
