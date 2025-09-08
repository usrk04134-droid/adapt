#include "../src/horizontal_tracker.h"

#include <doctest/doctest.h>

#include <optional>

#include "macs/macs_groove.h"
#include "macs/macs_point.h"

using tracking::HorizontalTracker;
using tracking::HorizontalTrackingMode;
using tracking::HorizontalTrackingReference;

// NOLINTBEGIN(*-magic-numbers)

TEST_SUITE("HorizontalTracker") {
  TEST_CASE("Basic tracking") {
    auto const joint = macs::Groove(
        macs::Point{.horizontal = 50.0, .vertical = 20.0}, macs::Point{.horizontal = 40.0, .vertical = 15.0},
        macs::Point{.horizontal = 30.0, .vertical = 10.0}, macs::Point{.horizontal = 30.0, .vertical = 10.0},
        macs::Point{.horizontal = 30.0, .vertical = 10.0}, macs::Point{.horizontal = 20.0, .vertical = 15.0},
        macs::Point{.horizontal = 10.0, .vertical = 20.0});

    auto tracker = HorizontalTracker(HorizontalTrackingMode::LEFT);
    tracker.SetTrackingReference(HorizontalTrackingReference::BOTTOM);
    tracker.SetOffset(4.);
    tracker.SetJoint(joint);
    auto target_pos_res = tracker.GetHorizontalMove();
    CHECK(target_pos_res.has_value());
    CHECK_EQ(target_pos_res.value(), 36.0);  // abw1 - offset
  }

  TEST_CASE("Track before setting slice or offset") {
    auto tracker        = HorizontalTracker(HorizontalTrackingMode::LEFT);
    auto target_pos_res = tracker.GetHorizontalMove();
    CHECK(!target_pos_res.has_value());
    tracker.SetOffset(4.0);

    target_pos_res = tracker.GetHorizontalMove();
    CHECK(!target_pos_res.has_value());
  }

  TEST_CASE("Switching modes") {
    auto const joint = macs::Groove(
        macs::Point{.horizontal = 50.0, .vertical = 20.0}, macs::Point{.horizontal = 40.0, .vertical = 15.0},
        macs::Point{.horizontal = 30.0, .vertical = 10.0}, macs::Point{.horizontal = 30.0, .vertical = 10.0},
        macs::Point{.horizontal = 30.0, .vertical = 10.0}, macs::Point{.horizontal = 20.0, .vertical = 15.0},
        macs::Point{.horizontal = 10.0, .vertical = 20.0});

    auto tracker = HorizontalTracker(HorizontalTrackingMode::LEFT);
    tracker.SetTrackingReference(HorizontalTrackingReference::BOTTOM);
    tracker.SetOffset(4.);
    tracker.SetJoint(joint);

    auto target_pos_res = tracker.GetHorizontalMove();
    CHECK(target_pos_res.has_value());
    CHECK_EQ(target_pos_res.value(), 36.0);  // abw1 - offset

    tracker.SetTrackingMode(HorizontalTrackingMode::MIDDLE);
    target_pos_res = tracker.GetHorizontalMove();
    CHECK(target_pos_res.has_value());
    CHECK_EQ(target_pos_res.value(), 26.0);  // middle of abw1 and abw5 - offset

    tracker.SetTrackingMode(HorizontalTrackingMode::RIGHT);
    target_pos_res = tracker.GetHorizontalMove();
    CHECK(target_pos_res.has_value());
    CHECK_EQ(target_pos_res.value(), 24.0);  // abw5 + offset
  }

  TEST_CASE("Tracking narrow joint") {
    // When joint is smaller than offset*2, the position shoud be the center of the groove.
    auto const joint = macs::Groove(
        macs::Point{.horizontal = 50.0, .vertical = 40.0}, macs::Point{.horizontal = 27.0, .vertical = 10.0},
        macs::Point{.horizontal = 26.0, .vertical = 8.0}, macs::Point{.horizontal = 25.0, .vertical = 7.0},
        macs::Point{.horizontal = 24.0, .vertical = 8.0}, macs::Point{.horizontal = 23.0, .vertical = 10.0},
        macs::Point{.horizontal = 0.0, .vertical = 40.0});

    auto tracker = HorizontalTracker(HorizontalTrackingMode::LEFT);
    tracker.SetTrackingReference(HorizontalTrackingReference::BOTTOM);
    tracker.SetOffset(4.);
    tracker.SetJoint(joint);

    auto target_pos_res = tracker.GetHorizontalMove();
    CHECK(target_pos_res.has_value());
    CHECK_EQ(target_pos_res.value(), 25.0);  // middle

    tracker.SetTrackingMode(HorizontalTrackingMode::RIGHT);
    target_pos_res = tracker.GetHorizontalMove();
    CHECK(target_pos_res.has_value());
    CHECK_EQ(target_pos_res.value(), 25.0);  // middle
  }

  TEST_CASE("Tracking with TOP reference") {
    auto const joint = macs::Groove(
        macs::Point{.horizontal = 50.0, .vertical = 20.0}, macs::Point{.horizontal = 40.0, .vertical = 15.0},
        macs::Point{.horizontal = 30.0, .vertical = 10.0}, macs::Point{.horizontal = 30.0, .vertical = 10.0},
        macs::Point{.horizontal = 30.0, .vertical = 10.0}, macs::Point{.horizontal = 20.0, .vertical = 15.0},
        macs::Point{.horizontal = 10.0, .vertical = 20.0});

    auto tracker = HorizontalTracker(HorizontalTrackingMode::LEFT);
    tracker.SetTrackingReference(HorizontalTrackingReference::TOP);
    tracker.SetOffset(4.);
    tracker.SetJoint(joint);

    auto target_pos_res = tracker.GetHorizontalMove();
    CHECK(target_pos_res.has_value());
    CHECK_EQ(target_pos_res.value(), 46.0);  // abw0 - offset

    tracker.SetTrackingMode(HorizontalTrackingMode::MIDDLE);
    target_pos_res = tracker.GetHorizontalMove();
    CHECK(target_pos_res.has_value());
    CHECK_EQ(target_pos_res.value(), 26.0);  // middle of abw0 and abw6 - offset

    tracker.SetTrackingMode(HorizontalTrackingMode::RIGHT);
    target_pos_res = tracker.GetHorizontalMove();
    CHECK(target_pos_res.has_value());
    CHECK_EQ(target_pos_res.value(), 14.0);  // abw6 + offset
  }
}

// NOLINTEND(*-magic-numbers)
