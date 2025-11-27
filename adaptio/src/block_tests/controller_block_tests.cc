#include <doctest/doctest.h>

#include "controller/controller_data.h"
#include "helpers/helpers_abp_parameters.h"
#include "helpers/helpers_joint_geometry.h"
#include "helpers/helpers_mfx.h"
#include "helpers/helpers_settings.h"

using controller::AdaptioInput;
using controller::AxisInput;
using controller::TrackInput;

namespace {
const uint32_t TRACKING_MODE_RIGHT = 2;

const uint32_t SEQUENCE_NONE         = 0;
const uint32_t SEQUENCE_TRACKING     = 1;
const uint32_t SEQUENCE_AUTO_WELDING = 2;
}  // namespace

TEST_SUITE("Controller Tests") {
  TEST_CASE("Start joint tracking") {
    MultiFixture mfx;

    StoreSettings(mfx.Main(), TestSettings{.use_edge_sensor = false}, true);
    StoreDefaultJointGeometryParams(mfx.Main());
    CHECK_EQ(SEQUENCE_NONE, mfx.Ctrl().Mock()->adaptio_output.get_active_sequence_type());

    AdaptioInput adaptio_input;
    adaptio_input.set_commands_start(true);
    adaptio_input.set_sequence_type(SEQUENCE_TRACKING);
    mfx.Ctrl().Sut()->OnAdaptioInputUpdate(adaptio_input);
    mfx.PlcDataUpdate();

    CHECK(mfx.Ctrl().Mock()->adaptio_output.get_status_ready_for_tracking());
    CHECK(mfx.Ctrl().Mock()->adaptio_output.get_status_active());
    CHECK_FALSE(mfx.Ctrl().Mock()->adaptio_output.get_status_error());
    CHECK(mfx.Ctrl().Mock()->track_output.get_status_active());
    CHECK_EQ(SEQUENCE_TRACKING, mfx.Ctrl().Mock()->adaptio_output.get_active_sequence_type());
  }

  TEST_CASE("Scanner error") {
    MultiFixture mfx;

    StoreSettings(mfx.Main(), TestSettings{.use_edge_sensor = false}, true);
    StoreDefaultJointGeometryParams(mfx.Main());

    AdaptioInput data{};
    data.set_commands_start(true);
    data.set_sequence_type(SEQUENCE_TRACKING);
    mfx.Ctrl().Sut()->OnAdaptioInputUpdate(data);
    mfx.PlcDataUpdate();

    // Receive StartScanner
    REQUIRE_MESSAGE(mfx.Main().Scanner()->Receive<common::msg::scanner::Start>(), "No Start msg received");
    // Respond with failure to trigger scanner error
    mfx.Main().Scanner()->Dispatch(common::msg::scanner::StartRsp{.success = false});

    CHECK(mfx.Ctrl().Mock()->adaptio_output.get_status_active());
    CHECK(mfx.Ctrl().Mock()->adaptio_output.get_status_error());
    CHECK_EQ(SEQUENCE_NONE, mfx.Ctrl().Mock()->adaptio_output.get_active_sequence_type());

    // Clear the error by sending stop
    AdaptioInput input_stop{};
    input_stop.set_commands_stop(true);
    mfx.Ctrl().Sut()->OnAdaptioInputUpdate(input_stop);
    mfx.PlcDataUpdate();

    CHECK(mfx.Ctrl().Mock()->adaptio_output.get_status_ready());
    CHECK_FALSE(mfx.Ctrl().Mock()->adaptio_output.get_status_active());
    CHECK_FALSE(mfx.Ctrl().Mock()->adaptio_output.get_status_error());
  }

  TEST_CASE("Start ABP failure") {
    MultiFixture mfx;

    StoreSettings(mfx.Main(), TestSettings{.use_edge_sensor = false}, true);
    StoreDefaultJointGeometryParams(mfx.Main());

    // Check ready bits
    CHECK(mfx.Ctrl().Mock()->adaptio_output.get_status_ready_for_tracking());
    CHECK_FALSE(mfx.Ctrl().Mock()->adaptio_output.get_status_ready_for_abp());

    // Check active and error bits
    CHECK_FALSE(mfx.Ctrl().Mock()->adaptio_output.get_status_active());
    CHECK_FALSE(mfx.Ctrl().Mock()->adaptio_output.get_status_error());

    AdaptioInput data{};
    data.set_commands_start(true);
    data.set_sequence_type(SEQUENCE_AUTO_WELDING);
    mfx.Ctrl().Sut()->OnAdaptioInputUpdate(data);
    mfx.PlcDataUpdate();

    // Check active and error bit
    CHECK(mfx.Ctrl().Mock()->adaptio_output.get_status_active());
    CHECK(mfx.Ctrl().Mock()->adaptio_output.get_status_error());
    CHECK_EQ(SEQUENCE_NONE, mfx.Ctrl().Mock()->adaptio_output.get_active_sequence_type());
  }

  TEST_CASE("Start ABP success") {
    MultiFixture mfx;

    StoreSettings(mfx.Main(), TestSettings{.use_edge_sensor = false}, true);
    StoreDefaultJointGeometryParams(mfx.Main());
    StoreDefaultABPParams(mfx.Main());

    AdaptioInput start_data{};
    AxisInput axis_data{};
    start_data.set_commands_start(true);
    mfx.Ctrl().Sut()->OnAdaptioInputUpdate(start_data);
    axis_data.set_status_homed(true);
    mfx.Ctrl().Sut()->OnWeldAxisInputUpdate(axis_data);
    mfx.PlcDataUpdate();

    // Check ready bits
    CHECK(mfx.Ctrl().Mock()->adaptio_output.get_status_ready_for_tracking());
    CHECK(mfx.Ctrl().Mock()->adaptio_output.get_status_ready_for_abp());

    AdaptioInput data{};
    data.set_commands_start(true);
    data.set_sequence_type(SEQUENCE_AUTO_WELDING);
    mfx.Ctrl().Sut()->OnAdaptioInputUpdate(data);
    mfx.PlcDataUpdate();

    // Check active and error bit
    CHECK(mfx.Ctrl().Mock()->adaptio_output.get_status_active());
    CHECK_EQ(SEQUENCE_AUTO_WELDING, mfx.Ctrl().Mock()->adaptio_output.get_active_sequence_type());
  }

  TEST_CASE("JT to ABP to JT") {
    MultiFixture mfx;

    StoreSettings(mfx.Main(), TestSettings{.use_edge_sensor = false}, true);
    StoreDefaultJointGeometryParams(mfx.Main());
    StoreDefaultABPParams(mfx.Main());

    AxisInput axis_data{};
    axis_data.set_status_homed(true);
    mfx.Ctrl().Sut()->OnWeldAxisInputUpdate(axis_data);
    mfx.PlcDataUpdate();

    {
      AdaptioInput data{};
      data.set_commands_start(true);
      data.set_sequence_type(SEQUENCE_TRACKING);
      mfx.Ctrl().Sut()->OnAdaptioInputUpdate(data);
      mfx.PlcDataUpdate();
      CHECK_EQ(SEQUENCE_TRACKING, mfx.Ctrl().Mock()->adaptio_output.get_active_sequence_type());
    }

    {
      // Start ABP
      AdaptioInput data{};
      data.set_commands_start(true);
      data.set_sequence_type(SEQUENCE_AUTO_WELDING);
      mfx.Ctrl().Sut()->OnAdaptioInputUpdate(data);
      mfx.PlcDataUpdate();
      CHECK_EQ(SEQUENCE_AUTO_WELDING, mfx.Ctrl().Mock()->adaptio_output.get_active_sequence_type());
    }

    {
      AdaptioInput data{};
      data.set_commands_start(true);
      data.set_sequence_type(SEQUENCE_TRACKING);
      mfx.Ctrl().Sut()->OnAdaptioInputUpdate(data);
      mfx.PlcDataUpdate();
      CHECK_EQ(SEQUENCE_TRACKING, mfx.Ctrl().Mock()->adaptio_output.get_active_sequence_type());
    }
  }

  TEST_CASE("No Tracking without start") {
    MultiFixture mfx;

    TrackInput track_data{};
    track_data.set_joint_tracking_mode(TRACKING_MODE_RIGHT);
    mfx.Ctrl().Sut()->OnTrackingInputUpdate(track_data);

    AdaptioInput data{};
    data.set_commands_start(false);
    data.set_sequence_type(SEQUENCE_TRACKING);
    mfx.Ctrl().Sut()->OnAdaptioInputUpdate(data);
    mfx.PlcDataUpdate();

    CHECK_FALSE(mfx.Ctrl().Mock()->adaptio_output.get_status_active());
    CHECK_FALSE(mfx.Ctrl().Mock()->adaptio_output.get_status_error());
    CHECK_FALSE(mfx.Ctrl().Mock()->track_output.get_status_active());
    CHECK_EQ(SEQUENCE_NONE, mfx.Ctrl().Mock()->adaptio_output.get_active_sequence_type());
  }

  TEST_CASE("No Error without start") {
    MultiFixture mfx;

    AdaptioInput data{};
    data.set_commands_start(false);
    data.set_sequence_type(SEQUENCE_TRACKING);
    mfx.Ctrl().Sut()->OnAdaptioInputUpdate(data);
    mfx.PlcDataUpdate();

    CHECK_FALSE(mfx.Ctrl().Mock()->adaptio_output.get_status_active());
    CHECK_FALSE(mfx.Ctrl().Mock()->adaptio_output.get_status_error());
    CHECK_FALSE(mfx.Ctrl().Mock()->track_output.get_status_active());
    CHECK_EQ(SEQUENCE_NONE, mfx.Ctrl().Mock()->adaptio_output.get_active_sequence_type());
  }
}
