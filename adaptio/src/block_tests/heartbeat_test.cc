// NOLINTBEGIN(*-magic-numbers, *-optional-access)

#include <doctest/doctest.h>

#include "block_tests/helpers/helpers_settings.h"
#include "controller/controller_data.h"
#include "helpers/helpers.h"
#include "helpers/helpers_joint_geometry.h"
#include "helpers/helpers_mfx.h"

using controller::AdaptioInput;

const uint32_t HEARTBEAT_1 = 100;
const uint32_t HEARTBEAT_2 = 101;

const uint32_t HEARTBEAT_SUPERVISION_TIME_PLUS_DELTA_WITHIN_LIMIT = 250;
const uint32_t HEARTBEAT_SUPERVISION_TIME_PLUS_DELTA_TIMEOUT      = 501;

TEST_SUITE("Heartbeat") {
  TEST_CASE("Joint_tracking_continues_heartbeat_ok") {
    MultiFixture mfx;
    AdaptioInput adaptio_input;

    StoreSettings(mfx.Main(), TestSettings{.use_edge_sensor = false}, true);
    StoreDefaultJointGeometryParams(mfx.Main());

    mfx.Ctrl().Sut()->SuperviseHeartbeat();

    TrackingPreconditions(mfx);

    // Tracking start with heartbeat
    adaptio_input.set_heartbeat(HEARTBEAT_1);
    adaptio_input.set_commands_start(true);
    adaptio_input.set_sequence_type(1);
    mfx.Ctrl().Sut()->OnAdaptioInputUpdate(adaptio_input);
    mfx.PlcDataUpdate();

    CHECK_EQ(mfx.Ctrl().Mock()->adaptio_output.get_active_sequence_type(), 1);
    CHECK_EQ(mfx.Ctrl().Mock()->adaptio_output.get_heartbeat(), HEARTBEAT_1);

    // Heartbeat ok
    adaptio_input.set_heartbeat(HEARTBEAT_2);
    mfx.Ctrl().Sut()->OnAdaptioInputUpdate(adaptio_input);

    mfx.Main().GetClockNowFuncWrapper()->StepSteadyClock(
        std::chrono::milliseconds{HEARTBEAT_SUPERVISION_TIME_PLUS_DELTA_TIMEOUT});
    mfx.PlcDataUpdate();

    CHECK_EQ(mfx.Ctrl().Mock()->adaptio_output.get_heartbeat(), HEARTBEAT_2);
    CHECK_EQ(mfx.Ctrl().Mock()->adaptio_output.get_active_sequence_type(), 1);
  }

  TEST_CASE("Joint_tracking_stop_after_500ms_when_heartbeat_fail") {
    MultiFixture mfx;
    AdaptioInput adaptio_input;

    StoreSettings(mfx.Main(), TestSettings{.use_edge_sensor = false}, true);
    StoreDefaultJointGeometryParams(mfx.Main());

    mfx.Ctrl().Sut()->SuperviseHeartbeat();
    TrackingPreconditions(mfx);

    // Tracking start with heartbeat
    adaptio_input.set_heartbeat(HEARTBEAT_1);
    adaptio_input.set_commands_start(true);
    adaptio_input.set_sequence_type(1);
    mfx.Ctrl().Sut()->OnAdaptioInputUpdate(adaptio_input);
    mfx.PlcDataUpdate();
    CHECK_EQ(mfx.Ctrl().Mock()->adaptio_output.get_active_sequence_type(), 1);
    CHECK_EQ(mfx.Ctrl().Mock()->adaptio_output.get_heartbeat(), HEARTBEAT_1);

    // Heartbeat fail to increase
    mfx.Ctrl().Sut()->OnAdaptioInputUpdate(adaptio_input);
    mfx.PlcDataUpdate();

    CHECK_EQ(mfx.Ctrl().Mock()->adaptio_output.get_active_sequence_type(), 1);

    // Heartbeat fail to increase wait time <500ms
    mfx.Main().GetClockNowFuncWrapper()->StepSteadyClock(
        std::chrono::milliseconds{HEARTBEAT_SUPERVISION_TIME_PLUS_DELTA_WITHIN_LIMIT});
    mfx.PlcDataUpdate();

    CHECK_EQ(mfx.Ctrl().Mock()->adaptio_output.get_active_sequence_type(), 1);

    // Heartbeat fail to increase  wait time >500ms  JT stop
    mfx.Main().GetClockNowFuncWrapper()->StepSteadyClock(
        std::chrono::milliseconds{HEARTBEAT_SUPERVISION_TIME_PLUS_DELTA_TIMEOUT});
    mfx.PlcDataUpdate();

    CHECK_EQ(mfx.Ctrl().Mock()->adaptio_output.get_active_sequence_type(), 0);
    CHECK_EQ(mfx.Ctrl().Mock()->adaptio_output.get_status_error(), 1);
    CHECK_EQ(mfx.Ctrl().Mock()->track_output.get_status_error(), 1);
  }
}
