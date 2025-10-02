#include "weld_control/src/confident_slice_buffer.h"

#include <doctest/doctest.h>

#include "common/groove/groove.h"

// NOLINTBEGIN(*-magic-numbers)
namespace weld_control {

TEST_SUITE("ConfidentSliceBuffer") {
  TEST_CASE("JsonRoundtrip") {
    auto original_slice = ConfidentSliceData{
        .edge_position = 10.5,
        .groove        = common::groove::Groove({.horizontal = 75., .vertical = 25.}, {.horizontal = 25., .vertical = -25.},
                                      {.horizontal = 12.5, .vertical = -25.}, {.horizontal = 0., .vertical = -25.},
                                      {.horizontal = -12.5, .vertical = -25.}, {.horizontal = -25, .vertical = -25.},
                                      {.horizontal = -75, .vertical = 25.})};

    auto json               = original_slice.ToJson();
    auto restored_slice_opt = ConfidentSliceData::FromJson(json);

    REQUIRE(restored_slice_opt.has_value());
    auto restored_slice = restored_slice_opt.value();

    REQUIRE(original_slice.edge_position == doctest::Approx(restored_slice.edge_position));

    for (auto i = 0; i < common::groove::ABW_POINTS; ++i) {
      REQUIRE(original_slice.groove[i].horizontal == doctest::Approx(restored_slice.groove[i].horizontal));
      REQUIRE(original_slice.groove[i].vertical == doctest::Approx(restored_slice.groove[i].vertical));
    }

    REQUIRE(original_slice.groove.Area() == doctest::Approx(restored_slice.groove.Area()));
    REQUIRE(original_slice.groove.TopWidth() == doctest::Approx(restored_slice.groove.TopWidth()));
  }
}

}  // namespace weld_control
// NOLINTEND(*-magic-numbers)
