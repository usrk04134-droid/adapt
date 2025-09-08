#include "bead_control/src/weld_position_data_storage.h"

#include <doctest/doctest.h>

#include <numbers>

#include "macs/macs_point.h"

// NOLINTBEGIN(*-magic-numbers, misc-include-cleaner)

TEST_SUITE("Test weld position data storage") {
  TEST_CASE("Test no data") {
    bead_control::WeldPositionDataStorage storage(bead_control::MAX_BUFFER_SIZE);

    auto const slice1 = macs::Groove({.horizontal = 1.0, .vertical = 1.1}, {.horizontal = 2.0, .vertical = 2.1},
                                     {.horizontal = 3.0, .vertical = 3.1}, {.horizontal = 4.0, .vertical = 4.1},
                                     {.horizontal = 5.0, .vertical = 5.1}, {.horizontal = 6.0, .vertical = 6.1},
                                     {.horizontal = 7.0, .vertical = 7.1});

    storage.Store(1.0, {.weld_object_lin_velocity = 2.0, .groove = slice1, .weld_system1 = {}});

    // Angle outside of range
    auto data = storage.Get(1.1, 1.3);
    CHECK(data.Size() == 0);
  }
  TEST_CASE("Test wrapped angle") {
    bead_control::WeldPositionDataStorage storage(bead_control::MAX_BUFFER_SIZE);

    auto const slice1 = macs::Groove({.horizontal = 1.0, .vertical = 1.1}, {.horizontal = 2.0, .vertical = 2.1},
                                     {.horizontal = 3.0, .vertical = 3.1}, {.horizontal = 4.0, .vertical = 4.1},
                                     {.horizontal = 5.0, .vertical = 5.1}, {.horizontal = 6.0, .vertical = 6.1},
                                     {.horizontal = 7.0, .vertical = 7.1});
    auto const slice2 = macs::Groove({.horizontal = 11.0, .vertical = 11.1}, {.horizontal = 12.0, .vertical = 12.1},
                                     {.horizontal = 13.0, .vertical = 13.1}, {.horizontal = 14.0, .vertical = 14.1},
                                     {.horizontal = 15.0, .vertical = 15.1}, {.horizontal = 16.0, .vertical = 16.1},
                                     {.horizontal = 17.0, .vertical = 17.1});
    auto const slice3 = macs::Groove({.horizontal = 21.0, .vertical = 21.1}, {.horizontal = 22.0, .vertical = 22.1},
                                     {.horizontal = 23.0, .vertical = 23.1}, {.horizontal = 24.0, .vertical = 24.1},
                                     {.horizontal = 25.0, .vertical = 25.1}, {.horizontal = 26.0, .vertical = 26.1},
                                     {.horizontal = 27.0, .vertical = 27.1});

    storage.Store(1.1, {.weld_object_lin_velocity = 2.0, .groove = slice1, .weld_system1 = {}});
    storage.Store(0.2, {.weld_object_lin_velocity = 2.0, .groove = slice2, .weld_system1 = {}});
    storage.Store(1.2, {.weld_object_lin_velocity = 2.0, .groove = slice3, .weld_system1 = {}});

    // Try to get data for current turn. The first position is not within the turn
    auto data     = storage.Get(0.0, 2 * std::numbers::pi);
    auto iterator = data.begin();
    CHECK(data.Size() == 2);

    CHECK(iterator->position == doctest::Approx(1.2));
    CHECK(iterator->data.groove[0].horizontal == doctest::Approx(21.0));
    CHECK(iterator->data.groove[0].vertical == doctest::Approx(21.1));
    CHECK(iterator->data.groove[1].horizontal == doctest::Approx(22.0));
    CHECK(iterator->data.groove[1].vertical == doctest::Approx(22.1));

    CHECK((++iterator)->position == doctest::Approx(0.2));
    CHECK(iterator->data.groove[0].horizontal == doctest::Approx(11.0));
    CHECK(iterator->data.groove[0].vertical == doctest::Approx(11.1));
    CHECK(iterator->data.groove[1].horizontal == doctest::Approx(12.0));
    CHECK(iterator->data.groove[1].vertical == doctest::Approx(12.1));
  }

  TEST_CASE("Test get large data") {
    bead_control::WeldPositionDataStorage storage(bead_control::MAX_BUFFER_SIZE);

    auto slice1 = macs::Groove({.horizontal = 1.0, .vertical = 1.1}, {.horizontal = 2.0, .vertical = 2.1},
                               {.horizontal = 3.0, .vertical = 3.1}, {.horizontal = 4.0, .vertical = 4.1},
                               {.horizontal = 5.0, .vertical = 5.1}, {.horizontal = 6.0, .vertical = 6.1},
                               {.horizontal = 7.0, .vertical = 7.1});
    auto slice2 = macs::Groove({.horizontal = 11.0, .vertical = 11.1}, {.horizontal = 12.0, .vertical = 12.1},
                               {.horizontal = 13.0, .vertical = 13.1}, {.horizontal = 14.0, .vertical = 14.1},
                               {.horizontal = 15.0, .vertical = 15.1}, {.horizontal = 16.0, .vertical = 16.1},
                               {.horizontal = 17.0, .vertical = 17.1});

    auto step = 2 * std::numbers::pi / 100000.0;
    // Store 100000 values for whole turn
    for (double i = 0.0; i < 2 * std::numbers::pi; i += step) {
      storage.Store(i, {.weld_object_lin_velocity = i + 1.0, .groove = slice1, .weld_system1 = {}});
    }
    // Store 100000 values for a new turn
    for (double i = 0.0; i < 2 * std::numbers::pi; i += step) {
      storage.Store(i, {.weld_object_lin_velocity = i + 1.0, .groove = slice2, .weld_system1 = {}});
    }

    // Get 80% of the values for the latest turn
    auto data = storage.Get(0.1 * 2 * std::numbers::pi, 0.9 * 2 * std::numbers::pi);
    CHECK(data.Size() == 80000);
    CHECK(data.begin()->data.groove[0].vertical == doctest::Approx(11.1));
    auto it = data.end() - 1;
    CHECK(it->data.groove[0].vertical == doctest::Approx(11.1));

    // Get 80% of the values for the previous turn
    data = storage.Get(1.1 * 2 * std::numbers::pi, 1.9 * 2 * std::numbers::pi);

    CHECK(data.Size() == 80000);
    CHECK(data.begin()->data.groove[0].vertical == doctest::Approx(1.1));
    it = data.end() - 1;
    CHECK(it->data.groove[0].vertical == doctest::Approx(1.1));
  }
}

// NOLINTEND(*-magic-numbers, misc-include-cleaner)
