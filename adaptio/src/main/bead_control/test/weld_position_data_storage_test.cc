#include "../src/weld_position_data_storage.h"

#include <doctest/doctest.h>

#include "common/groove/groove.h"

// NOLINTBEGIN(*-magic-numbers, misc-include-cleaner)
namespace bead_control {

TEST_SUITE("WeldPositionDataStorage") {
  TEST_CASE("constructor slice and size") {
    WeldPositionDataStorage storage(220000);

    auto const slice1 = common::groove::Groove({.horizontal = 1.0, .vertical = 1.1}, {.horizontal = 2.0, .vertical = 2.1},
                                     {.horizontal = 3.0, .vertical = 3.1}, {.horizontal = 4.0, .vertical = 4.1},
                                     {.horizontal = 5.0, .vertical = 5.1}, {.horizontal = 6.0, .vertical = 6.1},
                                     {.horizontal = 7.0, .vertical = 7.1});

    storage.Store(0.1, {.groove = slice1});

    auto const slice2 = common::groove::Groove({.horizontal = 11.0, .vertical = 11.1}, {.horizontal = 12.0, .vertical = 12.1},
                                     {.horizontal = 13.0, .vertical = 13.1}, {.horizontal = 14.0, .vertical = 14.1},
                                     {.horizontal = 15.0, .vertical = 15.1}, {.horizontal = 16.0, .vertical = 16.1},
                                     {.horizontal = 17.0, .vertical = 17.1});

    storage.Store(0.2, {.groove = slice2});

    auto const slice3 = common::groove::Groove({.horizontal = 21.0, .vertical = 21.1}, {.horizontal = 22.0, .vertical = 22.1},
                                     {.horizontal = 23.0, .vertical = 23.1}, {.horizontal = 24.0, .vertical = 24.1},
                                     {.horizontal = 25.0, .vertical = 25.1}, {.horizontal = 26.0, .vertical = 26.1},
                                     {.horizontal = 27.0, .vertical = 27.1});

    storage.Store(0.3, {.groove = slice3});

    // Get latest
    auto res = storage.Get(0.3, 0.3, true);
    CHECK(res.has_value());
    auto entry = res.value();
    CHECK(entry.second.groove[0].horizontal == doctest::Approx(21.0));

    // Get slice [0.1 0.2]
    res = storage.Get(0.1, 0.2);
    CHECK(res.has_value());

    WeldPositionDataStorage::Slice slice = res.value();
    CHECK(slice.Size() == 2);
    CHECK(slice[0].data.groove[0].horizontal == doctest::Approx(11.0));
  }

  TEST_CASE("slices of container") {
    WeldPositionDataStorage storage(10);

    auto slice1 = common::groove::Groove({.horizontal = 1.0, .vertical = 1.1}, {.horizontal = 2.0, .vertical = 2.1},
                               {.horizontal = 3.0, .vertical = 3.1}, {.horizontal = 4.0, .vertical = 4.1},
                               {.horizontal = 5.0, .vertical = 5.1}, {.horizontal = 6.0, .vertical = 6.1},
                               {.horizontal = 7.0, .vertical = 7.1});

    auto slice2 = common::groove::Groove({.horizontal = 11.0, .vertical = 11.1}, {.horizontal = 12.0, .vertical = 12.1},
                               {.horizontal = 13.0, .vertical = 13.1}, {.horizontal = 14.0, .vertical = 14.1},
                               {.horizontal = 15.0, .vertical = 15.1}, {.horizontal = 16.0, .vertical = 16.1},
                               {.horizontal = 17.0, .vertical = 17.1});

    auto res_slice = common::containers::PositionBuffer<common::groove::Groove>(10);
    res_slice.Store(0.3, slice1);
    res_slice.Store(0.4, slice2);

    auto res = res_slice.Slice(0.3, 0.3);

    CHECK(res.Size() == 1);

    auto res1 = res[0].data;

    auto res_slice1 = res_slice.Slice(0.3, 0.4);

    CHECK(res_slice1.Size() == 2);

    CHECK(res_slice1[0].data[0].horizontal == doctest::Approx(1.0));
    CHECK(res_slice1[1].data[0].horizontal == doctest::Approx(11.0));
  }
}

}  // namespace bead_control
// NOLINTEND(*-magic-numbers, misc-include-cleaner)
