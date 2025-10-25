#include "controller/controller_data.h"
#include "helpers/helpers.h"
#include "helpers/helpers_abp_parameters.h"
#include "helpers/helpers_calibration_v2.h"
#include "helpers/helpers_joint_geometry.h"
#include "helpers/helpers_settings.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access)

#include <doctest/doctest.h>
#include <fmt/core.h>

using controller::AxisInput;
using controller::TrackInput;

TEST_SUITE("MultiblockReadyState") {
  TEST_CASE("tracking_not_ready_on_weld_calibration") {
    MultiFixture mfx;

    StoreSettings(mfx.Main(), TestSettings{.use_edge_sensor = false}, true);
    StoreDefaultJointGeometryParams(mfx.Main());

    CHECK(mfx.Ctrl().Mock()->adaptio_output.get_status_ready_for_tracking());

    WeldObjectCalStart(mfx.Main(), 4.0, 25.0, 1000.0);
    mfx.PlcDataUpdate();
    CHECK_FALSE(mfx.Ctrl().Mock()->adaptio_output.get_status_ready_for_tracking());
  }

  TEST_CASE("abp_ready_1") {
    MultiFixture mfx;
    AxisInput axis_data{};

    StoreSettings(mfx.Main(), TestSettings{.use_edge_sensor = false}, true);
    StoreDefaultJointGeometryParams(mfx.Main());

    CHECK(mfx.Ctrl().Mock()->adaptio_output.get_status_ready_for_tracking());
    CHECK_FALSE(mfx.Ctrl().Mock()->adaptio_output.get_status_ready_for_abp());

    StoreDefaultABPParams(mfx.Main());
    axis_data.set_status_homed(true);
    mfx.Ctrl().Sut()->OnWeldAxisInputUpdate(axis_data);

    mfx.PlcDataUpdate();

    CHECK(mfx.Ctrl().Mock()->adaptio_output.get_status_ready_for_abp());
  }

  TEST_CASE("abp_jt_ready_with_edge_sensor") {
    MultiFixture mfx;
    AxisInput axis_data{};
    TrackInput tracking_data{};

    StoreDefaultJointGeometryParams(mfx.Main());

    StoreDefaultABPParams(mfx.Main());
    StoreSettings(mfx.Main(), TestSettings{.use_edge_sensor = true}, true);

    axis_data.set_status_homed(true);
    mfx.Ctrl().Sut()->OnWeldAxisInputUpdate(axis_data);
    mfx.PlcDataUpdate();

    CHECK_FALSE(mfx.Ctrl().Mock()->adaptio_output.get_status_ready_for_tracking());

    tracking_data.set_status_edge_tracker_value_valid(true);
    mfx.Ctrl().Sut()->OnTrackingInputUpdate(tracking_data);

    axis_data.set_position(0.0);
    axis_data.set_velocity(0.0);
    tracking_data.set_weld_object_radius(100.0);
    mfx.Ctrl().Sut()->OnWeldAxisInputUpdate(axis_data);
    mfx.Ctrl().Sut()->OnTrackingInputUpdate(tracking_data);

    mfx.PlcDataUpdate();
    CHECK(mfx.Ctrl().Mock()->adaptio_output.get_status_ready_for_abp());
  }
}
