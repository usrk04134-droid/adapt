#include <doctest/doctest.h>

#include <nlohmann/json_fwd.hpp>

#include "helpers/helpers.h"
#include "helpers/helpers_weld_process_parameters.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)

const bool EXPECT_OK  = true;
const bool EXPECT_NOK = false;

const nlohmann::json WPP_1 = {
    {"name",           "FillType1"},
    {"method",         "dc"       },
    {"regulationType", "cc"       },
    {"startAdjust",    10         },
    {"startType",      "scratch"  },
    {"voltage",        24.5       },
    {"current",        150.0      },
    {"wireSpeed",      12.5       },
    {"iceWireSpeed",   0.0        },
    {"acFrequency",    60.0       },
    {"acOffset",       1.2        },
    {"acPhaseShift",   0.5        },
    {"craterFillTime", 2.0        },
    {"burnBackTime",   1.0        }
};

const nlohmann::json WPP_2 = {
    {"name",           "FillType2"},
    {"method",         "ac"       },
    {"regulationType", "cc"       },
    {"startAdjust",    10         },
    {"startType",      "direct"   },
    {"voltage",        24.5       },
    {"current",        100.0      },
    {"wireSpeed",      12.5       },
    {"iceWireSpeed",   0.0        },
    {"acFrequency",    60.0       },
    {"acOffset",       1.2        },
    {"acPhaseShift",   0.5        },
    {"craterFillTime", 2.0        },
    {"burnBackTime",   1.0        }
};

TEST_SUITE("WeldProcessParameters") {
  TEST_CASE("add_get") {
    TestFixture fixture;
    fixture.StartApplication();

    /* Store Weld process parameters successfully */
    AddWeldProcessParameters(fixture, WPP_1, EXPECT_OK);
    AddWeldProcessParameters(fixture, WPP_2, EXPECT_OK);

    auto expected_wpp_1  = WPP_1;
    expected_wpp_1["id"] = 1;  // added when stored to db
    auto expected_wpp_2  = WPP_2;
    expected_wpp_2["id"] = 2;

    auto expected_array = nlohmann::json::array();
    expected_array.push_back(expected_wpp_1);
    expected_array.push_back(expected_wpp_2);

    CHECK(CheckWeldProcessParametersEqual(fixture, expected_array));
  }

  TEST_CASE("add_conflict") {
    TestFixture fixture;
    fixture.StartApplication();

    /* Store Weld process parameters twice, name conflict */
    AddWeldProcessParameters(fixture, WPP_1, EXPECT_OK);
    AddWeldProcessParameters(fixture, WPP_1, EXPECT_NOK);
  }

  TEST_CASE("add_invalid") {
    TestFixture fixture;
    fixture.StartApplication();

    /* Invalid WPP*/
    auto invalid_wpp      = WPP_1;
    invalid_wpp["method"] = "superQuick";
    AddWeldProcessParameters(fixture, invalid_wpp, EXPECT_NOK);
  }
}
// NOLINTEND(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)
