#include "helpers/helpers.h"
#include "helpers/helpers_abp_parameters.h"
#include "helpers/helpers_joint_geometry.h"
#include "helpers/helpers_mfx.h"
#include "helpers/helpers_settings.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access)

#include <doctest/doctest.h>

#include <chrono>

const std::chrono::milliseconds MS6000{6000};

TEST_SUITE("ScannerBasic") {
  TEST_CASE("initiate_scanner_fixture") {
    MultiFixture mfx;
    mfx.SetupScanner("40297730");

    StoreSettings(mfx.Main(), TestSettings{.use_edge_sensor = false}, true);
    StoreDefaultJointGeometryParams(mfx.Main());

    TrackingPreconditions(mfx);
    TrackingStart(mfx);

    mfx.Scan().RepeatToFillMedianBuffer("1738243625597.tiff");
    mfx.Scan().TriggerScannerData();

    CHECK_NE(mfx.Ctrl().Mock()->axis_x_output.get_position(), 0.0);
  }

  TEST_CASE("detection_error") {
    MultiFixture mfx;
    mfx.SetupScanner("40402058");

    StoreSettings(mfx.Main(), TestSettings{.use_edge_sensor = false}, true);
    StoreDefaultJointGeometryParams(mfx.Main());

    TrackingPreconditions(mfx);
    TrackingStart(mfx);

    // A completely black image triggers a groove detection error after 5 sec
    mfx.Scan().RepeatToFillMedianBuffer("1754561083373.tiff");
    mfx.Scan().TriggerScannerData();

    mfx.Main().GetClockNowFuncWrapper()->StepSteadyClock(MS6000);
    mfx.Scan().TriggerScannerData();

    CHECK_EQ(mfx.Ctrl().Mock()->adaptio_output.get_status_error(), 1);
  }
}
