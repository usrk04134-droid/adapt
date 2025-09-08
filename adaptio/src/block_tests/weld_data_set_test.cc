
#include <doctest/doctest.h>

#include <nlohmann/json_fwd.hpp>
#include <string>

#include "helpers.h"
#include "helpers_weld_data_set.h"
#include "helpers_weld_program.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)

static const bool EXPECT_OK   = true;
static const bool EXPECT_FAIL = false;

TEST_SUITE("WeldDataSet") {
  TEST_CASE("weld_data_set_web_hmi") {
    TestFixture fixture;
    fixture.StartApplication();

    /* no stored weld data sets -> empty json array */
    nlohmann::json payload2 = nlohmann::json::array();
    CHECK_EQ(GetWeldDataSets(fixture), payload2);

    /* check that the application can handle removal of a non existing data set */
    CHECK(RemoveWeldDataSet(fixture, 1, EXPECT_FAIL));
    CHECK(RemoveWeldDataSet(fixture, 2, EXPECT_FAIL));
    CHECK(RemoveWeldDataSet(fixture, 10, EXPECT_FAIL));

    /* add one weld data set */
    CHECK(AddWeldDataSet(fixture, "root1", 1, 2, EXPECT_OK));

    /* check that the weld data set was added successfully */
    payload2.push_back(nlohmann::json({
        {"id",       1      },
        {"name",     "root1"},
        {"ws1WppId", 1      },
        {"ws2WppId", 2      }
    }));
    CHECK_EQ(GetWeldDataSets(fixture), payload2);

    /* add two more weld data sets and check that they were added*/
    CHECK(AddWeldDataSet(fixture, "fill2", 3, 4, true));
    CHECK(AddWeldDataSet(fixture, "fill3", 5, 6, true));

    payload2.push_back(nlohmann::json({
        {"id",       2      },
        {"name",     "fill2"},
        {"ws1WppId", 3      },
        {"ws2WppId", 4      }
    }));
    payload2.push_back(nlohmann::json({
        {"id",       3      },
        {"name",     "fill3"},
        {"ws1WppId", 5      },
        {"ws2WppId", 6      }
    }));
    CHECK_EQ(GetWeldDataSets(fixture), payload2);

    /* remove one of the weld data sets and check that it was removed */
    CHECK(RemoveWeldDataSet(fixture, 2, EXPECT_OK));
    payload2.erase(payload2.begin() + 1, payload2.begin() + 2);
    CHECK_EQ(GetWeldDataSets(fixture), payload2);

    /* remove the two remaining weld data sets and check that they were removed */
    CHECK(RemoveWeldDataSet(fixture, 3, EXPECT_OK));
    CHECK(RemoveWeldDataSet(fixture, 1, EXPECT_OK));
    payload2.clear();
    CHECK_EQ(GetWeldDataSets(fixture), payload2);
  }

  TEST_CASE("weld_data_set_validation_failed") {
    TestFixture fixture;
    fixture.StartApplication();

    /* add a weld data set that will fail the validation - invalid name */
    CHECK(AddWeldDataSet(fixture, "", 7, 8, EXPECT_FAIL));

    /* no stored weld data sets -> empty json array */
    nlohmann::json payload2 = nlohmann::json::array();
    CHECK_EQ(GetWeldDataSets(fixture), payload2);
  }

  TEST_CASE("weld_data_set_fetch_from_database") {
    /* The database will be populated when started the first time. The second time the application is started the data
     * will be read from the database */
    TestFixture fixture;
    fixture.StartApplication();
    CHECK(AddWeldDataSet(fixture, "weld_params1", 1, 2, EXPECT_OK));

    fixture.StopApplication();

    // Instantiate a new application with the populated database
    fixture.StartApplication();
    nlohmann::json expected = nlohmann::json::array();
    expected.push_back(nlohmann::json({
        {"id",       1             },
        {"name",     "weld_params1"},
        {"ws1WppId", 1             },
        {"ws2WppId", 2             }
    }));
    CHECK_EQ(GetWeldDataSets(fixture), expected);
  }

  TEST_CASE("remove_used_weld_data_set") {
    TestFixture fixture;
    fixture.StartApplication();
    /* Check that it is not possible to remove a weld-data-set that is
     * used by a weld-program */

    CHECK(AddWeldDataSet(fixture, "root1", 1, 2, EXPECT_OK));

    const nlohmann::json program{
        {"name",     "test_program"                              },
        {"grooveId", 1                                           },
        {"layers",   {{{"layerNumber", 0}, {"weldDataSetId", 1}}}}
    };

    CHECK(StoreWeldProgram(fixture, program, EXPECT_OK));

    auto expected  = program;
    expected["id"] = 1;  // added when stored to db
    CHECK(CheckWeldProgramsEqual(fixture, nlohmann::json::array({expected})));

    CHECK(RemoveWeldDataSet(fixture, 1, EXPECT_FAIL));
  }
}
// NOLINTEND(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)
