#include <doctest/doctest.h>

#include <nlohmann/json_fwd.hpp>
#include <vector>

#include "helpers.h"
#include "helpers_abp_parameters.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)

const bool EXPECT_OK           = true;
const bool EXPECT_NOK          = false;
const double WALL_OFFSET       = 3.7;
const double BEAD_OVERLAP      = 42.0;
const double STEP_UP_VALUE     = 99.1;
const double K_GAIN            = 2.;
const double HEAT_INPUT_MIN    = 2.1;
const double HEAT_INPUT_MAX    = 2.9;
const double CURRENT_MIN       = 650.;
const double CURRENT_MAX       = 700.;
const double WELD_SPEED_MIN    = 95.;
const double WELD_SPEED_MAX    = 105.;
const double BEAD_SWITCH_ANGLE = 15.;
const double CAP_CORNER_OFFSET = 1.2;
const int CAP_BEADS            = 3;
const double CAP_INIT_DEPTH    = 7.0;

namespace {

struct TestParams {
  double wall_offset{WALL_OFFSET};
  double bead_overlap{BEAD_OVERLAP};
  double step_up_value{STEP_UP_VALUE};
  double k_gain{K_GAIN};
  double heat_input_min{HEAT_INPUT_MIN};
  double heat_input_max{HEAT_INPUT_MAX};
  double current_min{CURRENT_MIN};
  double current_max{CURRENT_MAX};
  double weld_speed_min{WELD_SPEED_MIN};
  double weld_speed_max{WELD_SPEED_MAX};
  double bead_switch_angle{BEAD_SWITCH_ANGLE};
  std::vector<double> step_up_limits = {25.0, 30.0, 35.0, 40.0, 50.0};
  double cap_corner_offset{CAP_CORNER_OFFSET};
  int cap_beads{CAP_BEADS};
  double cap_init_depth{CAP_INIT_DEPTH};
};

auto AbpParametersPayload(const TestParams& params) -> auto {
  auto json_step_up_limits = nlohmann::json::array();
  for (auto step_up_limit : params.step_up_limits) {
    json_step_up_limits.push_back(step_up_limit);
  }

  return nlohmann::json({
      {"wallOffset",         params.wall_offset      },
      {"beadOverlap",        params.bead_overlap     },
      {"stepUpValue",        params.step_up_value    },
      {"kGain",              params.k_gain           },
      {"heatInput",
       {
           {"min", params.heat_input_min},
           {"max", params.heat_input_max},
       }                                             },
      {"weldSystem2Current",
       {
           {"min", params.current_min},
           {"max", params.current_max},
       }                                             },
      {"weldSpeed",
       {
           {"min", params.weld_speed_min},
           {"max", params.weld_speed_max},
       }                                             },
      {"beadSwitchAngle",    params.bead_switch_angle},
      {"stepUpLimits",       json_step_up_limits     },
      {"capCornerOffset",    params.cap_corner_offset},
      {"capBeads",           params.cap_beads        },
      {"capInitDepth",       params.cap_init_depth   },
  });
}
}  // namespace

TEST_SUITE("ABPParameters") {
  TEST_CASE("abp_store_update_get") {
    TestFixture fixture;
    fixture.StartApplication();

    /* Store ABP parameters successfully */
    auto const payload1 = AbpParametersPayload(TestParams{});
    StoreABPParams(fixture, payload1, EXPECT_OK);
    CheckABPParamsEqual(fixture, payload1);

    /* Update ABP parameters successfully */
    auto params               = TestParams{};
    params.wall_offset       += 1.;
    params.bead_overlap      += 1.;
    params.step_up_value     += 1.;
    params.k_gain            += 1.;
    params.heat_input_min    += 1.;
    params.heat_input_max    += 1.;
    params.current_min       += 1.;
    params.current_max       += 1.;
    params.weld_speed_min    += 1.;
    params.weld_speed_max    += 1.;
    params.bead_switch_angle += 1.;
    params.step_up_limits.push_back(55.0);
    params.cap_corner_offset += 0.5;
    params.cap_beads         += 1;
    params.cap_init_depth    -= 0.1;

    auto const payload2 = AbpParametersPayload(params);
    StoreABPParams(fixture, payload2, EXPECT_OK);
    CheckABPParamsEqual(fixture, payload2);

    /* Legacy request where new parameters are not included */
    auto const payload5_req = nlohmann::json({
        {"wallOffset",  WALL_OFFSET  },
        {"beadOverlap", BEAD_OVERLAP },
        {"stepUpValue", STEP_UP_VALUE}
    });
    StoreABPParams(fixture, payload5_req, EXPECT_OK);

    auto const payload5_rsp = nlohmann::json({
        {"wallOffset",         WALL_OFFSET      },
        {"beadOverlap",        BEAD_OVERLAP     },
        {"stepUpValue",        STEP_UP_VALUE    },
        {"kGain",              K_GAIN           },
        {"heatInput",
         {
             {"min", 0.},
             {"max", 0.},
         }                                      },
        {"weldSystem2Current",
         {
             {"min", 0.},
             {"max", 0.},
         }                                      },
        {"weldSpeed",
         {
             {"min", 0.},
             {"max", 0.},
         }                                      },
        {"beadSwitchAngle",    BEAD_SWITCH_ANGLE},
        {"capCornerOffset",    0.0              },
        {"capBeads",           2                },
        {"capInitDepth",       0.0              },
    });
    CheckABPParamsEqual(fixture, payload5_rsp);
  }

  TEST_CASE("invalid_input") {
    TestFixture fixture;
    fixture.StartApplication();

    auto const tests = {
        TestParams{.wall_offset = 0.},
        TestParams{.bead_overlap = -.1},
        TestParams{.heat_input_min = -.1},
        TestParams{.heat_input_max = -.1},
        TestParams{.current_min = -.1},
        TestParams{.current_max = -.1},
        TestParams{.heat_input_max = HEAT_INPUT_MIN - 0.1},
        TestParams{.current_max = CURRENT_MIN - 0.1},
        TestParams{.bead_switch_angle = 4.9},
        TestParams{.bead_switch_angle = 30.1},
        TestParams{.weld_speed_min = -.1},
        TestParams{.weld_speed_max = -.1},
        TestParams{.weld_speed_max = WELD_SPEED_MIN - 0.1},

        /* the step_up_limits must positive be >= prev value */
        TestParams{.step_up_limits = {25.0, 20.0}},
        // TestParams{.step_up_limits = {-5.0}},

        /* new cap parameters validation */
        TestParams{.cap_beads = 1},
        TestParams{.cap_beads = -1},
        TestParams{.cap_init_depth = -0.1},
    };

    /* Store valid parameters */
    auto const payload1 = AbpParametersPayload(TestParams{});
    StoreABPParams(fixture, payload1, EXPECT_OK);

    for (auto const& test : tests) {
      auto const payload2 = AbpParametersPayload(test);
      StoreABPParams(fixture, payload2, EXPECT_NOK);

      /* Parameters are not overwritten */
      CheckABPParamsEqual(fixture, payload1);
    }
  }
}
// NOLINTEND(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)
