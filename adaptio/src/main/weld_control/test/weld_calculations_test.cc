#include "weld_control/src/weld_calculations.h"

#include <doctest/doctest.h>

#include <cmath>

#include "common/math/math.h"

// NOLINTBEGIN(*-magic-numbers, misc-include-cleaner)

namespace weld_control {
TEST_SUITE("WeldCalculations") {
  TEST_CASE("WeldSystem2Current") {
    struct TestParams {
      WeldCalc::CalculateAdaptivityInput params;
      WeldCalc::CalculateAdaptivityOutput result;
    };

    auto const
        tests =
            {
                /* Perfect slice - use mid value */
                TestParams{
                           .params = {.weld_current_ratio = 1.0,
                               .weld_speed_ratio   = 1.0,
                               .heat_input_min     = 2.3,
                               .heat_input_max     = 2.8,
                               .ws1                = {.current = 700., .voltage = 29.},
                               .ws2                = {.current_min = 675., .current_max = 725., .voltage = 31.},
                               .weld_object        = {.weld_speed_min = common::math::CmMinToMmSec(90.),
                                                      .weld_speed_max = common::math::CmMinToMmSec(110.)}},
                           .result =
                        {
                            .weld_speed  = common::math::CmMinToMmSec(100.),
                            .ws2_current = 700.,
                        }, },

                /* Large slice - High current and low weld-speed - Limited by max current and min weld-speed */
                TestParams{
                           .params = {.weld_current_ratio = 1.2,
                               .weld_speed_ratio   = 0.8,
                               .heat_input_min     = 2.3,
                               .heat_input_max     = 2.9,
                               .ws1                = {.current = 700., .voltage = 29.},
                               .ws2                = {.current_min = 675., .current_max = 725., .voltage = 31.},
                               .weld_object        = {.weld_speed_min = common::math::CmMinToMmSec(90.),
                                                      .weld_speed_max = common::math::CmMinToMmSec(110.)}},
                           .result =
                        {
                            .weld_speed  = common::math::CmMinToMmSec(90.),
                            .ws2_current = 725.,
                        }, },

                /* Small slice - Low current and high weld-speed - Limited by min current and max weld-speed */
                TestParams{
                           .params = {.weld_current_ratio = 0.8,
                               .weld_speed_ratio   = 1.2,
                               .heat_input_min     = 2.2,
                               .heat_input_max     = 2.8,
                               .ws1                = {.current = 700., .voltage = 29.},
                               .ws2                = {.current_min = 675., .current_max = 725., .voltage = 31.},
                               .weld_object        = {.weld_speed_min = common::math::CmMinToMmSec(90.),
                                                      .weld_speed_max = common::math::CmMinToMmSec(110.)}},
                           .result =
                        {
                            .weld_speed  = common::math::CmMinToMmSec(110.),
                            .ws2_current = 675.,
                        }, },

                /* Large slice - High current and low weld-speed - Limited by max heat input */
                TestParams{
                           .params = {.weld_current_ratio = 1.2,
                               .weld_speed_ratio   = 0.8,
                               .heat_input_min     = 2.3,
                               .heat_input_max     = 2.7,
                               .ws1                = {.current = 700., .voltage = 29.},
                               .ws2                = {.current_min = 675., .current_max = 725., .voltage = 31.},
                               .weld_object        = {.weld_speed_min = common::math::CmMinToMmSec(90.),
                                                      .weld_speed_max = common::math::CmMinToMmSec(110.)}},
                           .result =
                        {
                            .weld_speed  = common::math::CmMinToMmSec(92.5),
                            .ws2_current = 687.3,
                        }, },

                /* Small slice - Low current and high weld-speed - Limited by min heat input */
                TestParams{
                           .params = {.weld_current_ratio = 0.8,
                               .weld_speed_ratio   = 1.2,
                               .heat_input_min     = 2.3,
                               .heat_input_max     = 2.8,
                               .ws1                = {.current = 700., .voltage = 29.},
                               .ws2                = {.current_min = 675., .current_max = 725., .voltage = 31.},
                               .weld_object        = {.weld_speed_min = common::math::CmMinToMmSec(90.),
                                                      .weld_speed_max = common::math::CmMinToMmSec(110.)}},
                           .result =
                        {
                            .weld_speed  = common::math::CmMinToMmSec(108.8),
                            .ws2_current = 690.017,
                        }, },

                /* Large slice - no weld-speed adaptivity - High current - Limited by max heat input */
                TestParams{
                           .params = {.weld_current_ratio = 1.2,
                               .weld_speed_ratio   = 0.8,
                               .heat_input_min     = 2.3,
                               .heat_input_max     = 2.5,
                               .ws1                = {.current = 700., .voltage = 29.},
                               .ws2                = {.current_min = 675., .current_max = 725., .voltage = 31.},
                               .weld_object        = {.weld_speed_min = common::math::CmMinToMmSec(100.),
                                                      .weld_speed_max = common::math::CmMinToMmSec(100.)}},
                           .result =
                        {
                            .weld_speed  = common::math::CmMinToMmSec(100.),
                            .ws2_current = 689.247,
                        }, },
    };

    for (auto const &test : tests) {
      auto const result = WeldCalc::CalculateAdaptivity(test.params);
      REQUIRE(result.ws2_current == doctest::Approx(test.result.ws2_current).epsilon(0.001));
      REQUIRE(result.weld_speed == doctest::Approx(test.result.weld_speed).epsilon(0.001));
    }
  }
}

}  // namespace weld_control
// NOLINTEND(*-magic-numbers, misc-include-cleaner)
