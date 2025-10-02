#include "../src/vertical_tracker.h"

#include <doctest/doctest.h>

#include <optional>

#include "common/groove/groove.h"

using tracking::VerticalTracker;

// NOLINTBEGIN(*-magic-numbers)

TEST_SUITE("VerticalTracker") {
  TEST_CASE("Basic tracking") {
    auto const line = common::groove::Groove({.horizontal = 70.0, .vertical = 20.0}, {.horizontal = 60.0, .vertical = 15.0},
                                   {.horizontal = 50.0, .vertical = 12.5}, {.horizontal = 40.0, .vertical = 10.0},
                                   {.horizontal = 30.0, .vertical = 12.5}, {.horizontal = 20.0, .vertical = 15.0},
                                   {.horizontal = 10.0, .vertical = 20.0});
    double current_hor_pos = 35;

    auto tracker = VerticalTracker();
    tracker.SetOffset(30.);
    tracker.SetLine(line);
    auto target_pos_res = tracker.GetVerticalMove(current_hor_pos);
    CHECK(target_pos_res.has_value());
    CHECK_EQ(target_pos_res.value(), 41.25);  // Middle of point height between abw-point 3 and 4 + offset
  }

  TEST_CASE("Track before setting line or offset") {
    double current_hor_pos = 25;

    auto tracker = VerticalTracker();

    auto target_pos_res = tracker.GetVerticalMove(current_hor_pos);
    CHECK(!target_pos_res.has_value());

    tracker.SetOffset(4.0);
    target_pos_res = tracker.GetVerticalMove(current_hor_pos);
    CHECK(!target_pos_res.has_value());

    tracker = VerticalTracker();

    auto const line = common::groove::Groove({.horizontal = 50.0, .vertical = 20.0}, {.horizontal = 40.0, .vertical = 15.0},
                                   {.horizontal = 30.0, .vertical = 10.0}, {.horizontal = 27.0, .vertical = 10.0},
                                   {.horizontal = 23.0, .vertical = 10.0}, {.horizontal = 20.0, .vertical = 15.0},
                                   {.horizontal = 10.0, .vertical = 20.0});
    tracker.SetLine(line);
    target_pos_res = tracker.GetVerticalMove(current_hor_pos);
    CHECK(!target_pos_res.has_value());
  }
}
// NOLINTEND(*-magic-numbers)
