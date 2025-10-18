#include <doctest/doctest.h>

#include "helpers/helpers.h"
#include "helpers/helpers_settings.h"

const bool EXPECT_OK = true;

TEST_SUITE("Settings") {
  TEST_CASE("store_update_get") {
    TestFixture fixture;
    fixture.StartApplication();

    /* Store settings successfully */
    auto settings = TestSettings{.use_edge_sensor = true, .edge_sensor_placement = "right"};
    StoreSettings(fixture, settings, EXPECT_OK);
    CheckSettingsEqual(fixture, settings);

    settings = TestSettings{.use_edge_sensor = false, .edge_sensor_placement = "left"};
    StoreSettings(fixture, settings, EXPECT_OK);
    CheckSettingsEqual(fixture, settings);
  }

  TEST_CASE("edge_sensor_placement_values") {
    TestFixture fixture;
    fixture.StartApplication();

    /* Test left placement */
    auto settings = TestSettings{.use_edge_sensor = true, .edge_sensor_placement = "left"};
    StoreSettings(fixture, settings, EXPECT_OK);
    CheckSettingsEqual(fixture, settings);

    /* Test right placement */
    settings = TestSettings{.use_edge_sensor = true, .edge_sensor_placement = "right"};
    StoreSettings(fixture, settings, EXPECT_OK);
    CheckSettingsEqual(fixture, settings);
  }
}
