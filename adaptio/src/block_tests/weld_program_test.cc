#include <doctest/doctest.h>

#include <nlohmann/json_fwd.hpp>
#include <string>

#include "helpers/helpers.h"
#include "helpers/helpers_weld_data_set.h"
#include "helpers/helpers_weld_program.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)

static const bool EXPECT_OK   = true;
static const bool EXPECT_FAIL = false;

TEST_SUITE("WeldProgram") {
  TEST_CASE("store_update_get") {
    TestFixture fixture;
    fixture.StartApplication();
    /* Store weld-program including a layer with weld-data-set that does not exist */
    nlohmann::json layers = nlohmann::json::array();
    layers.push_back({
        {"layerNumber",   0},
        {"weldDataSetId", 1}
    });

    nlohmann::json const payload{
        {"name",     "program1"},
        {"grooveId", 1         },
        {"layers",   layers    }
    };
    CHECK(StoreWeldProgram(fixture, payload, EXPECT_FAIL));

    /* Add the weld-data-set (gets wds_id = 1) and try again -> success */
    CHECK(AddWeldDataSet(fixture, "root1", 1, 2, EXPECT_OK));
    CHECK(StoreWeldProgram(fixture, payload, EXPECT_OK));

    /* Check that the layer was added to the weld-program */
    auto expected  = payload;
    expected["id"] = 1;  // added when stored to db
    CHECK(CheckWeldProgramsEqual(fixture, nlohmann::json::array({expected})));

    /* Store weld-program including a layer with weld-data-set that does not exist */
    layers.push_back(nlohmann::json{
        {"weldDataSetId", 2}
    });
    nlohmann::json const payload2{
        {"layers", layers}
    };
    CHECK(StoreWeldProgram(fixture, payload2, EXPECT_FAIL));

    /* Check that the weld-program was not changed */
    CHECK(CheckWeldProgramsEqual(fixture, nlohmann::json::array({expected})));
  }

  TEST_CASE("weld_program_fetch_from_database") {
    /* The database will be populated when started the first time. The second time the application is started the data
     * will be read from the database */

    nlohmann::json layers = nlohmann::json::array();
    layers.push_back({
        {"layerNumber",   0},
        {"weldDataSetId", 1}
    });

    nlohmann::json const payload{
        {"name",     "program1"},
        {"grooveId", 1         },
        {"layers",   layers    }
    };

    TestFixture fixture;
    fixture.StartApplication();
    CHECK(AddWeldDataSet(fixture, "root1", 1, 2, EXPECT_OK));
    CHECK(StoreWeldProgram(fixture, payload, EXPECT_OK));

    fixture.StartApplication();
    auto expected  = payload;
    expected["id"] = 1;  // added when stored to db
    CHECK(CheckWeldProgramsEqual(fixture, nlohmann::json::array({expected})));
  }
}
// NOLINTEND(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)
