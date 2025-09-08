#include "bead_control/src/groove_fit.h"

#include <doctest/doctest.h>

#include <cmath>
#include <numbers>

#include "bead_control/src/weld_position_data_storage.h"
#include "macs/macs_groove.h"

// NOLINTBEGIN(*-magic-numbers)

namespace bead_control {

TEST_SUITE("GrooveFit") {
  static auto const GROOVES_EQUAL = [](auto const groove1, auto const groove2) {
    for (auto j = 0; j < macs::ABW_POINTS; ++j) {
      if (groove1[j].horizontal != doctest::Approx(groove2[j].horizontal) ||
          groove1[j].vertical != doctest::Approx(groove2[j].vertical)) {
        return false;
      }
    }
    return true;
  };
  TEST_CASE("groove10Samples3rdDegreePolynomial") {
    auto const groove = macs::Groove({.horizontal = 75.0, .vertical = 25.0}, {.horizontal = 25.0, .vertical = -25.0},
                                     {.horizontal = 12.5, .vertical = -25.0}, {.horizontal = 0.0, .vertical = -25.0},
                                     {.horizontal = -12.5, .vertical = -25.0}, {.horizontal = -25.0, .vertical = -25.0},
                                     {.horizontal = -75.0, .vertical = 25.0});

    auto const steps = 10;
    for (auto type : {GrooveFit::Type::POLYNOMIAL, GrooveFit::Type::FOURIER}) {
      bead_control::WeldPositionDataStorage storage(steps);

      for (auto i = 0; i < steps; ++i) {
        auto const pos = 2 * std::numbers::pi / steps * i;
        storage.Store(pos, {.groove = groove});
      }

      auto fg = GrooveFit(storage.Get(0, 2 * std::numbers::pi), type, 3, 0);

      /* use more positions comapred to when the groove fit was created */
      for (auto i = 0; i < 3 * steps; ++i) {
        auto const pos        = 2 * std::numbers::pi / steps * i;
        auto const groove_fit = fg.Fit(pos);

        CHECK(groove_fit.IsValid());
        CHECK(GROOVES_EQUAL(groove, groove_fit));
      }
    }
  }

  TEST_CASE("groove5Samples8thDegreePolynominal") {
    auto const groove = macs::Groove({.horizontal = 75.0, .vertical = 25.0}, {.horizontal = 25.0, .vertical = -25.0},
                                     {.horizontal = 12.5, .vertical = -25.0}, {.horizontal = 0.0, .vertical = -25.0},
                                     {.horizontal = -12.5, .vertical = -25.0}, {.horizontal = -25.0, .vertical = -25.0},
                                     {.horizontal = -75.0, .vertical = 25.0});
    auto const steps  = 10;
    for (auto type : {GrooveFit::Type::POLYNOMIAL, GrooveFit::Type::FOURIER}) {
      bead_control::WeldPositionDataStorage storage(steps);

      for (auto i = 0; i < steps; ++i) {
        auto const pos = 2 * std::numbers::pi / steps * i;
        storage.Store(pos, {.groove = groove});
      }

      /* use 5 samples when creating the groove fit */
      auto fg = GrooveFit(storage.Get(0, 2 * std::numbers::pi), type, 8, 5);

      /* use more positions comapred to when the groove fit was created */
      for (auto i = 0; i < 3 * steps; ++i) {
        auto const pos        = 2 * std::numbers::pi / steps * i;
        auto const groove_fit = fg.Fit(pos);

        CHECK(groove_fit.IsValid());
        CHECK(GROOVES_EQUAL(groove, groove_fit));
      }
    }
  }
}
}  // namespace bead_control

// NOLINTEND(*-magic-numbers)
