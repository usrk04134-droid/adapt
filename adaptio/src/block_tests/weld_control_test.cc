#include <doctest/doctest.h>

#include <chrono>
#include <cstdint>
#include <numbers>
#include <optional>

#include "common/math/math.h"
#include "common/messages/kinematics.h"
#include "common/messages/management.h"
#include "common/messages/scanner.h"
#include "common/messages/weld_system.h"
#include "event_handler/event_codes.h"
#include "event_handler/src/event_types.h"
#include "helpers/helpers.h"
#include "helpers/helpers_abp_parameters.h"
#include "helpers/helpers_event_handling.h"
#include "helpers/helpers_joint_geometry.h"
#include "helpers/helpers_kinematics.h"
#include "helpers/helpers_settings.h"
#include "helpers/helpers_weld_control.h"
#include "helpers/helpers_weld_system.h"
#include "tracking/tracking_manager.h"
#include "weld_control/weld_control_types.h"
#include "weld_system_client/weld_system_types.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access)

namespace {
const float HORIZONTAL_OFFSET = 10.0;
const float VERTICAL_OFFSET   = 20.0;
}  // namespace

TEST_SUITE("WeldControl") {
  TEST_CASE("weld_control_status") {
    TestFixture fixture;
    fixture.StartApplication();

    StoreDefaultJointGeometryParams(fixture);

    CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "idle"});
    // Start Joint tracking
    common::msg::management::TrackingStart start_joint_tracking_msg{
        .joint_tracking_mode = static_cast<uint32_t>(tracking::TrackingMode::TRACKING_LEFT_HEIGHT),
        .horizontal_offset   = HORIZONTAL_OFFSET,
        .vertical_offset     = VERTICAL_OFFSET};
    fixture.Management()->Dispatch(start_joint_tracking_msg);

    // Receive StartScanner
    CHECK(fixture.Scanner()->Receive<common::msg::scanner::Start>());

    CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "jt"});
  }

  TEST_CASE("input") {
    TestFixture fixture;
    fixture.StartApplication();

    StoreSettings(fixture, TestSettings{.use_edge_sensor = false}, true);
    StoreDefaultJointGeometryParams(fixture);

    // Start Joint tracking
    common::msg::management::TrackingStart start_joint_tracking_msg{
        .joint_tracking_mode = static_cast<uint32_t>(tracking::TrackingMode::TRACKING_LEFT_HEIGHT),
        .horizontal_offset   = HORIZONTAL_OFFSET,
        .vertical_offset     = VERTICAL_OFFSET};
    fixture.Management()->Dispatch(start_joint_tracking_msg);

    // Receive StartScanner
    CHECK(fixture.Scanner()->Receive<common::msg::scanner::Start>());

    // ABW points on scanner interface
    fixture.Scanner()->Dispatch(fixture.ScannerData()->Get());

    // Receive GetPosition
    auto get_position = fixture.Kinematics()->Receive<common::msg::kinematics::GetSlidesPosition>();
    CHECK(get_position);

    // GetPosition response
    fixture.Kinematics()->Dispatch(common::msg::kinematics::GetSlidesPositionRsp{.client_id  = get_position->client_id,
                                                                                 .time_stamp = get_position->time_stamp,
                                                                                 .horizontal = -20,
                                                                                 .vertical   = 5});

    double const position = 1.23;
    double const distance = 1.0;
    double const velocity = 2.55;
    double const radius   = 3500;

    // Check GetWeldAxis request and dispatch response
    CheckAndDispatchGetWeldAxis(fixture, position, distance, velocity, radius);

    const common::msg::weld_system::GetWeldSystemDataRsp weld_system_status_rsp1{
        .voltage           = 30.123456,
        .current           = 200.0,
        .wire_lin_velocity = 12.1,
        .deposition_rate   = 3.5,
        .heat_input        = 120.0,
        .twin_wire         = true,
        .wire_diameter     = 1.1,
    };

    const common::msg::weld_system::GetWeldSystemDataRsp weld_system_status_rsp2{
        .voltage           = 31.0,
        .current           = 216.123456,
        .wire_lin_velocity = 10.1,
        .deposition_rate   = 4.5,
        .heat_input        = 125.0,
        .twin_wire         = false,
        .wire_diameter     = 1.2,
    };

    // Check GetWeldSystemStatus requests for both weld-systems and send response
    CheckAndDispatchWeldSystemDataRsp(fixture, weld_system::WeldSystemId::ID1, weld_system_status_rsp1);
    CheckAndDispatchWeldSystemDataRsp(fixture, weld_system::WeldSystemId::ID2, weld_system_status_rsp2);

    // Finally receive SetPosition
    CHECK(fixture.Kinematics()->Receive<common::msg::kinematics::SetSlidesPosition>());

    DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID1,
                                  common::msg::weld_system::OnWeldSystemStateChange::State::READY_TO_START);
    DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID2,
                                  common::msg::weld_system::OnWeldSystemStateChange::State::READY_TO_START);
    DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID1,
                                  common::msg::weld_system::OnWeldSystemStateChange::State::IN_WELDING_SEQUENCE);
    DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID2,
                                  common::msg::weld_system::OnWeldSystemStateChange::State::IN_WELDING_SEQUENCE);
    DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID1,
                                  common::msg::weld_system::OnWeldSystemStateChange::State::ARCING);
    DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID2,
                                  common::msg::weld_system::OnWeldSystemStateChange::State::ARCING);
  }

  TEST_CASE("input_no_edge_sensor") {
    TestFixture fixture;
    fixture.StartApplication();

    StoreDefaultJointGeometryParams(fixture);
    StoreSettings(fixture, TestSettings{.use_edge_sensor = false}, true);

    CheckEvents(fixture, {});

    StoreDefaultABPParams(fixture);
    DispatchKinematicsStateChange(fixture, common::msg::kinematics::StateChange::State::HOMED);
    CheckAndDispatchGetWeldAxis(fixture, 0.0, 0.0, 0.0, 100.0);

    common::msg::management::TrackingStart start_joint_tracking_msg{
        .joint_tracking_mode = static_cast<uint32_t>(tracking::TrackingMode::TRACKING_LEFT_HEIGHT),
        .horizontal_offset   = HORIZONTAL_OFFSET,
        .vertical_offset     = VERTICAL_OFFSET};
    fixture.Management()->Dispatch(start_joint_tracking_msg);

    CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "jt"});

    StartABP(fixture);
    CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "abp"});

    DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID1,
                                  common::msg::weld_system::OnWeldSystemStateChange::State::ARCING);
    DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID2,
                                  common::msg::weld_system::OnWeldSystemStateChange::State::ARCING);

    // Receive StartScanner
    CHECK(fixture.Scanner()->Receive<common::msg::scanner::Start>());

    // ABW points on scanner interface
    fixture.Scanner()->Dispatch(fixture.ScannerData()->Get());

    // Receive GetPosition
    auto get_position = fixture.Kinematics()->Receive<common::msg::kinematics::GetSlidesPosition>();
    CHECK(get_position);

    // GetPosition response
    fixture.Kinematics()->Dispatch(common::msg::kinematics::GetSlidesPositionRsp{.client_id  = get_position->client_id,
                                                                                 .time_stamp = get_position->time_stamp,
                                                                                 .horizontal = -20,
                                                                                 .vertical   = 5});
    // Check GetWeldAxis request and dispatch response
    CheckAndDispatchGetWeldAxis(fixture, 0.1, 0.1, 0.1, 2500);

    const common::msg::weld_system::GetWeldSystemDataRsp weld_system_status_rsp{
        .voltage           = 30.123456,
        .current           = 200.0,
        .wire_lin_velocity = 12.1,
        .deposition_rate   = 3.5,
        .heat_input        = 120.0,
        .twin_wire         = true,
        .wire_diameter     = 1.1,
    };

    // Check GetWeldSystemStatus requests for both weld-systems and send response
    CheckAndDispatchWeldSystemDataRsp(fixture, weld_system::WeldSystemId::ID1, weld_system_status_rsp);
    CheckAndDispatchWeldSystemDataRsp(fixture, weld_system::WeldSystemId::ID2, weld_system_status_rsp);
  }

  TEST_CASE("start_abp") {
    TestFixture fixture;
    fixture.StartApplication();

    StoreDefaultJointGeometryParams(fixture);

    CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "idle"});
    // Start Joint tracking
    common::msg::management::TrackingStart start_joint_tracking_msg{
        .joint_tracking_mode = static_cast<uint32_t>(tracking::TrackingMode::TRACKING_LEFT_HEIGHT),
        .horizontal_offset   = HORIZONTAL_OFFSET,
        .vertical_offset     = VERTICAL_OFFSET};
    fixture.Management()->Dispatch(start_joint_tracking_msg);

    // Receive StartScanner
    CHECK(fixture.Scanner()->Receive<common::msg::scanner::Start>());

    CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "jt"});

    StoreDefaultABPParams(fixture);
    DispatchKinematicsStateChange(fixture, common::msg::kinematics::StateChange::State::HOMED);
    DispatchKinematicsEdgeStateChange(fixture, common::msg::kinematics::EdgeStateChange::State::AVAILABLE);
    CheckAndDispatchGetWeldAxis(fixture, 0.0, 0.0, 0.0, 100.0);

    StartABP(fixture);
    CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "abp"});
  }

  TEST_CASE("input_validation") {
    TestFixture fixture;
    fixture.StartApplication();

    StoreDefaultJointGeometryParams(fixture);

    CheckEvents(fixture, {});

    StoreDefaultABPParams(fixture);
    DispatchKinematicsStateChange(fixture, common::msg::kinematics::StateChange::State::HOMED);
    DispatchKinematicsEdgeStateChange(fixture, common::msg::kinematics::EdgeStateChange::State::AVAILABLE);
    CheckAndDispatchGetWeldAxis(fixture, 0.0, 0.0, 0.0, 100.0);

    auto setup = [&fixture](weld_control::Mode mode) {
      StoreSettings(fixture, TestSettings{.use_edge_sensor = false}, true);
      common::msg::management::TrackingStart start_joint_tracking_msg{
          .joint_tracking_mode = static_cast<uint32_t>(tracking::TrackingMode::TRACKING_LEFT_HEIGHT),
          .horizontal_offset   = HORIZONTAL_OFFSET,
          .vertical_offset     = VERTICAL_OFFSET};
      fixture.Management()->Dispatch(start_joint_tracking_msg);

      CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "jt"});

      if (mode == weld_control::Mode::AUTOMATIC_BEAD_PLACEMENT) {
        StartABP(fixture);
        CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "abp"});
      }

      DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID1,
                                    common::msg::weld_system::OnWeldSystemStateChange::State::ARCING);
      DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID2,
                                    common::msg::weld_system::OnWeldSystemStateChange::State::ARCING);

      // Receive StartScanner
      CHECK(fixture.Scanner()->Receive<common::msg::scanner::Start>());
    };

    auto input = [&fixture](double position, double distance, double velocity, double radius) {
      // ABW points on scanner interface
      fixture.Scanner()->Dispatch(fixture.ScannerData()->Get());

      // Receive GetPosition
      auto get_position = fixture.Kinematics()->Receive<common::msg::kinematics::GetSlidesPosition>();
      CHECK(get_position);

      // GetPosition response
      fixture.Kinematics()->Dispatch(
          common::msg::kinematics::GetSlidesPositionRsp{.client_id  = get_position->client_id,
                                                        .time_stamp = get_position->time_stamp,
                                                        .horizontal = -20,
                                                        .vertical   = 5});
      // Check GetWeldAxis request and dispatch response
      CheckAndDispatchGetWeldAxis(fixture, position, distance, velocity, radius);

      const common::msg::weld_system::GetWeldSystemDataRsp weld_system_status_rsp{
          .voltage           = 30.123456,
          .current           = 200.0,
          .wire_lin_velocity = 12.1,
          .deposition_rate   = 3.5,
          .heat_input        = 120.0,
          .twin_wire         = true,
          .wire_diameter     = 1.1,
      };

      // Check GetWeldSystemStatus requests for both weld-systems and send response
      CheckAndDispatchWeldSystemDataRsp(fixture, weld_system::WeldSystemId::ID1, weld_system_status_rsp);
      CheckAndDispatchWeldSystemDataRsp(fixture, weld_system::WeldSystemId::ID2, weld_system_status_rsp);
    };

    // Check that the alert for goove detection timout was triggered
    auto const event_invalid_input = event::Event{
        .code = event::ABP_INVALID_INPUT,
    };

    auto const event_weld_axis_position = event::Event{
        .code = event::WELD_AXIS_INVALID_POSITION,
    };

    /* invalid weld-axis position */
    {
      setup(weld_control::Mode::JOINT_TRACKING);
      input(-0.1, -0.1, 0.012, 5000);
      CheckEvents(fixture, {event_invalid_input});
      CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "idle"});

      /* --- repeat test for ABP --- */

      setup(weld_control::Mode::AUTOMATIC_BEAD_PLACEMENT);
      input(-0.1, -0.1, 0.012, 5000);
      CheckEvents(fixture, {event_invalid_input});
      CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "idle"});
    }

    /* invalid weld-object radius */
    {
      setup(weld_control::Mode::JOINT_TRACKING);
      input(0.0, 0.0, 0.012, 0.0);
      CheckEvents(fixture, {event_invalid_input});
      CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "idle"});

      /* --- repeat test for ABP --- */

      setup(weld_control::Mode::AUTOMATIC_BEAD_PLACEMENT);
      input(0.0, 0.0, 0.012, -0.1);
      CheckEvents(fixture, {event_invalid_input});
      CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "idle"});
    }

    /* weld-axis position > 360 degrees but < 363 degrees - OK
     * followed by a postion > 363 degrees - NOK */
    {
      setup(weld_control::Mode::JOINT_TRACKING);
      input((2 * std::numbers::pi) + common::math::DegToRad(1.0), 0.0, 0.012, 750.0);
      CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "jt"});

      input((2 * std::numbers::pi) + common::math::DegToRad(4.0), 0.0, 0.012, 750.0);
      CheckEvents(fixture, {event_weld_axis_position});
      CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "idle"});

      /* --- repeat test for ABP --- */

      setup(weld_control::Mode::AUTOMATIC_BEAD_PLACEMENT);
      input((2 * std::numbers::pi) + common::math::DegToRad(1.0), 0.0, 0.012, 750.0);
      CHECK(GetWeldControlStatus(fixture).weld_control_mode.value() == "abp");

      input((2 * std::numbers::pi) + common::math::DegToRad(4.0), 0.0, 0.012, 750.0);
      CheckEvents(fixture, {event_weld_axis_position});
      CHECK(GetWeldControlStatus(fixture).weld_control_mode.value() == "idle");
    }

    /* welding state - weld-object position > 10mm from the last valid weld-object position in the reverse direction -
     * only valid for ABP */
    {
      auto const radius = 1000;
      setup(weld_control::Mode::AUTOMATIC_BEAD_PLACEMENT);
      input(common::math::DegToRad(100.0), 0.0, 0.012, radius);
      CHECK(GetWeldControlStatus(fixture).weld_control_mode.value() == "abp");

      input(common::math::DegToRad(100.0) - common::math::LinearToAngular(9, radius), 0.0, 0.012, radius);
      CHECK(GetWeldControlStatus(fixture).weld_control_mode.value() == "abp");

      input(common::math::DegToRad(100.0) - common::math::LinearToAngular(11, radius), 0.0, 0.012, radius);
      CheckEvents(fixture, {event_invalid_input});
      CHECK(GetWeldControlStatus(fixture).weld_control_mode.value() == "idle");
    }
  }

  TEST_CASE("arcing_lost_supervision") {
    TestFixture fixture;
    fixture.StartApplication();
    CheckEvents(fixture, {});

    StoreDefaultJointGeometryParams(fixture);
    StoreDefaultABPParams(fixture);
    DispatchKinematicsStateChange(fixture, common::msg::kinematics::StateChange::State::HOMED);
    DispatchKinematicsEdgeStateChange(fixture, common::msg::kinematics::EdgeStateChange::State::AVAILABLE);
    CheckAndDispatchGetWeldAxis(fixture, 0.0, 0.0, 0.0, 100.0);

    auto setup = [&fixture](weld_control::Mode mode) {
      StoreSettings(fixture, TestSettings{.use_edge_sensor = false}, true);
      common::msg::management::TrackingStart start_joint_tracking_msg{
          .joint_tracking_mode = static_cast<uint32_t>(tracking::TrackingMode::TRACKING_LEFT_HEIGHT),
          .horizontal_offset   = HORIZONTAL_OFFSET,
          .vertical_offset     = VERTICAL_OFFSET};
      fixture.Management()->Dispatch(start_joint_tracking_msg);

      CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "jt"});

      if (mode == weld_control::Mode::AUTOMATIC_BEAD_PLACEMENT) {
        StartABP(fixture);
        CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "abp"});
      }

      DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID1,
                                    common::msg::weld_system::OnWeldSystemStateChange::State::ARCING);
      DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID2,
                                    common::msg::weld_system::OnWeldSystemStateChange::State::ARCING);

      // Receive StartScanner
      CHECK(fixture.Scanner()->Receive<common::msg::scanner::Start>());
    };

    auto input = [&fixture](double position, double distance, double velocity, double radius) {
      // ABW points on scanner interface
      fixture.Scanner()->Dispatch(fixture.ScannerData()->Get());

      // Receive GetPosition
      auto get_position = fixture.Kinematics()->Receive<common::msg::kinematics::GetSlidesPosition>();
      CHECK(get_position);

      // GetPosition response
      fixture.Kinematics()->Dispatch(
          common::msg::kinematics::GetSlidesPositionRsp{.client_id  = get_position->client_id,
                                                        .time_stamp = get_position->time_stamp,
                                                        .horizontal = -20,
                                                        .vertical   = 5});
      // Check GetWeldAxis request and dispatch response
      CheckAndDispatchGetWeldAxis(fixture, position, distance, velocity, radius);

      const common::msg::weld_system::GetWeldSystemDataRsp weld_system_status_rsp{
          .voltage           = 30.123456,
          .current           = 200.0,
          .wire_lin_velocity = 12.1,
          .deposition_rate   = 3.5,
          .heat_input        = 120.0,
          .twin_wire         = true,
          .wire_diameter     = 1.1,
      };

      // Check GetWeldSystemStatus requests for both weld-systems and send response
      CheckAndDispatchWeldSystemDataRsp(fixture, weld_system::WeldSystemId::ID1, weld_system_status_rsp);
      CheckAndDispatchWeldSystemDataRsp(fixture, weld_system::WeldSystemId::ID2, weld_system_status_rsp);
    };

    auto const duration_lost_arcing_grace = fixture.Sut()->GetWeldControlConfig().supervision.arcing_lost_grace;

    auto const event_arcing_lost = event::Event{
        .code = event::ARCING_LOST,
    };

    /* test lost arcing on weld-system 1 */
    {
      setup(weld_control::Mode::AUTOMATIC_BEAD_PLACEMENT);
      input(0.0, 0.0, 0.012, 5000);
      CHECK(GetWeldControlStatus(fixture).weld_control_mode.value() == "abp");

      DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID1,
                                    common::msg::weld_system::OnWeldSystemStateChange::State::INIT);

      fixture.GetClockNowFuncWrapper()->StepSteadyClock(duration_lost_arcing_grace);
      input(0.1, 0.0, 0.012, 5000);
      CHECK(GetWeldControlStatus(fixture).weld_control_mode.value() == "abp");

      fixture.GetClockNowFuncWrapper()->StepSteadyClock(std::chrono::milliseconds(1));
      input(0.2, 0.0, 0.012, 5000);
      CHECK(GetWeldControlStatus(fixture).weld_control_mode.value() == "idle");
      CheckEvents(fixture, {event_arcing_lost});
    }

    /* test lost arcing on weld-system 2 */
    {
      setup(weld_control::Mode::AUTOMATIC_BEAD_PLACEMENT);
      input(0.0, 0.0, 0.012, 5000);
      CHECK(GetWeldControlStatus(fixture).weld_control_mode.value() == "abp");

      DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID2,
                                    common::msg::weld_system::OnWeldSystemStateChange::State::INIT);

      fixture.GetClockNowFuncWrapper()->StepSteadyClock(duration_lost_arcing_grace + std::chrono::milliseconds(1));
      input(0.1, 0.0, 0.012, 5000);
      CHECK(GetWeldControlStatus(fixture).weld_control_mode.value() == "idle");
      CheckEvents(fixture, {event_arcing_lost});
    }

    /* test lost arcing and regained on weld-system 1 - no event/stop */
    {
      setup(weld_control::Mode::AUTOMATIC_BEAD_PLACEMENT);
      input(0.0, 0.0, 0.012, 5000);
      CHECK(GetWeldControlStatus(fixture).weld_control_mode.value() == "abp");

      DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID1,
                                    common::msg::weld_system::OnWeldSystemStateChange::State::INIT);

      fixture.GetClockNowFuncWrapper()->StepSteadyClock(duration_lost_arcing_grace);
      input(0.1, 0.0, 0.012, 5000);
      CHECK(GetWeldControlStatus(fixture).weld_control_mode.value() == "abp");

      DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID1,
                                    common::msg::weld_system::OnWeldSystemStateChange::State::ARCING);

      fixture.GetClockNowFuncWrapper()->StepSteadyClock(std::chrono::milliseconds(10));
      input(0.2, 0.0, 0.012, 5000);
      CHECK(GetWeldControlStatus(fixture).weld_control_mode.value() == "abp");
    }
  }

  TEST_CASE("management_start_stop_abp") {
    TestFixture fixture;
    fixture.StartApplication();

    StoreDefaultJointGeometryParams(fixture);

    CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "idle"});

    // Start Joint tracking
    common::msg::management::TrackingStart start_joint_tracking_msg{
        .joint_tracking_mode = static_cast<uint32_t>(tracking::TrackingMode::TRACKING_LEFT_HEIGHT),
        .horizontal_offset   = HORIZONTAL_OFFSET,
        .vertical_offset     = VERTICAL_OFFSET};
    fixture.Management()->Dispatch(start_joint_tracking_msg);
    CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "jt"});

    StoreDefaultABPParams(fixture);
    DispatchKinematicsStateChange(fixture, common::msg::kinematics::StateChange::State::HOMED);
    DispatchKinematicsEdgeStateChange(fixture, common::msg::kinematics::EdgeStateChange::State::AVAILABLE);
    CheckAndDispatchGetWeldAxis(fixture, 0.0, 0.0, 0.0, 100.0);

    // Start and stop ABP
    fixture.Management()->Dispatch(common::msg::management::ABPStart{});
    CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "abp"});

    fixture.Management()->Dispatch(common::msg::management::ABPStop{});
    CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "jt"});
  }

  TEST_CASE("edge_sensor_not_available_when_in_idle") {
    TestFixture fixture;
    fixture.StartApplication();
    CheckEvents(fixture, {});

    StoreDefaultJointGeometryParams(fixture);
    StoreDefaultABPParams(fixture);
    DispatchKinematicsStateChange(fixture, common::msg::kinematics::StateChange::State::HOMED);
    DispatchKinematicsEdgeStateChange(fixture, common::msg::kinematics::EdgeStateChange::State::AVAILABLE);
    CheckAndDispatchGetWeldAxis(fixture, 0.0, 0.0, 0.0, 100.0);

    common::msg::management::TrackingStart start_joint_tracking_msg{
        .joint_tracking_mode = static_cast<uint32_t>(tracking::TrackingMode::TRACKING_LEFT_HEIGHT),
        .horizontal_offset   = HORIZONTAL_OFFSET,
        .vertical_offset     = VERTICAL_OFFSET};
    fixture.Management()->Dispatch(start_joint_tracking_msg);

    CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "jt"});

    StartABP(fixture);
    CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "abp"});

    DispatchKinematicsEdgeStateChange(fixture, common::msg::kinematics::EdgeStateChange::State::NOT_AVAILABLE);
  }

  TEST_CASE("ConfidenceLevels") {
    TestFixture fixture;
    fixture.StartApplication();
    CheckEvents(fixture, {});

    StoreDefaultJointGeometryParams(fixture);
    StoreDefaultABPParams(fixture);
    DispatchKinematicsStateChange(fixture, common::msg::kinematics::StateChange::State::HOMED);
    DispatchKinematicsEdgeStateChange(fixture, common::msg::kinematics::EdgeStateChange::State::AVAILABLE);
    CheckAndDispatchGetWeldAxis(fixture, 0.0, 0.0, 0.0, 100.0);

    auto setup = [&fixture](weld_control::Mode mode) {
      StoreSettings(fixture, TestSettings{.use_edge_sensor = false}, true);
      common::msg::management::TrackingStart start_joint_tracking_msg{
          .joint_tracking_mode = static_cast<uint32_t>(tracking::TrackingMode::TRACKING_LEFT_HEIGHT),
          .horizontal_offset   = HORIZONTAL_OFFSET,
          .vertical_offset     = VERTICAL_OFFSET};
      fixture.Management()->Dispatch(start_joint_tracking_msg);

      CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "jt"});

      if (mode == weld_control::Mode::AUTOMATIC_BEAD_PLACEMENT) {
        StartABP(fixture);
        CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "abp"});
      }

      DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID1,
                                    common::msg::weld_system::OnWeldSystemStateChange::State::ARCING);
      DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID2,
                                    common::msg::weld_system::OnWeldSystemStateChange::State::ARCING);

      // Receive StartScanner
      CHECK(fixture.Scanner()->Receive<common::msg::scanner::Start>());
    };

    auto input = [&fixture](common::msg::scanner::SliceData slice_data, std::chrono::milliseconds time_step,
                            bool stopped_or_no_previous_valid_data) {
      fixture.GetClockNowFuncWrapper()->StepSteadyClock(time_step);

      // ABW points on scanner interface
      fixture.Scanner()->Dispatch(slice_data);

      // Receive GetPosition
      auto get_position = fixture.Kinematics()->Receive<common::msg::kinematics::GetSlidesPosition>();
      CHECK(get_position);

      // GetPosition response
      fixture.Kinematics()->Dispatch(
          common::msg::kinematics::GetSlidesPositionRsp{.client_id  = get_position->client_id,
                                                        .time_stamp = get_position->time_stamp,
                                                        .horizontal = -20,
                                                        .vertical   = 5});
      if (!stopped_or_no_previous_valid_data) {
        // Check GetWeldAxis request and dispatch response
        CheckAndDispatchGetWeldAxis(fixture, 0.1, 0.0, 0.1, 2500);

        const common::msg::weld_system::GetWeldSystemDataRsp weld_system_status_rsp{
            .voltage           = 30.123456,
            .current           = 200.0,
            .wire_lin_velocity = 12.1,
            .deposition_rate   = 3.5,
            .heat_input        = 120.0,
            .twin_wire         = true,
            .wire_diameter     = 1.1,
        };

        // Check GetWeldSystemStatus requests for both weld-systems and send response
        CheckAndDispatchWeldSystemDataRsp(fixture, weld_system::WeldSystemId::ID1, weld_system_status_rsp);
        CheckAndDispatchWeldSystemDataRsp(fixture, weld_system::WeldSystemId::ID2, weld_system_status_rsp);
      }
    };

    auto const no_confidence_grace = std::chrono::duration_cast<std::chrono::milliseconds>(
        fixture.Sut()->GetWeldControlConfig().scanner_no_confidence_grace);
    auto const low_confidence_grace = std::chrono::duration_cast<std::chrono::milliseconds>(
        fixture.Sut()->GetWeldControlConfig().scanner_low_confidence_grace);
    auto const handover_grace =
        std::chrono::duration_cast<std::chrono::milliseconds>(fixture.Sut()->GetWeldControlConfig().handover_grace);

    auto const event_groove_detection_error = event::Event{
        .code = event::GROOVE_DETECTION_ERROR,
    };

    auto const event_handover_failed = event::Event{
        .code = event::HANDOVER_FAILED,
    };

    /* No confidence does not have a valid groove or line */
    auto no_conf_slice_data  = common::msg::scanner::SliceData{.confidence = common::msg::scanner::SliceConfidence::NO};
    auto low_conf_slice_data = fixture.ScannerData()->GetWithConfidence(common::msg::scanner::SliceConfidence::LOW);
    auto medium_conf_slice_data =
        fixture.ScannerData()->GetWithConfidence(common::msg::scanner::SliceConfidence::MEDIUM);
    auto high_conf_slice_data = fixture.ScannerData()->GetWithConfidence(common::msg::scanner::SliceConfidence::HIGH);

    SUBCASE("NO confidence") {
      // Check that event is triggered and adaptio is stopped for scanner input with NO confidence level
      setup(weld_control::Mode::AUTOMATIC_BEAD_PLACEMENT);

      input(high_conf_slice_data, std::chrono::milliseconds(0), false);

      input(no_conf_slice_data, std::chrono::milliseconds(0), false);

      input(no_conf_slice_data, no_confidence_grace, false);
      CHECK(!fixture.Management()->Receive<common::msg::management::TrackingStoppedGrooveDataTimeout>());

      input(no_conf_slice_data, std::chrono::milliseconds(1), false);
      CHECK(fixture.Management()->Receive<common::msg::management::TrackingStoppedGrooveDataTimeout>());

      CheckEvents(fixture, {event_groove_detection_error});
    }

    SUBCASE("NO confidence - cleared") {
      // Check NO confidence handling is cleared when receiving a slice with MEDIUM confidence level
      setup(weld_control::Mode::AUTOMATIC_BEAD_PLACEMENT);

      input(high_conf_slice_data, std::chrono::milliseconds(0), false);

      input(no_conf_slice_data, std::chrono::milliseconds(0), false);

      input(no_conf_slice_data, no_confidence_grace, false);
      CHECK(!fixture.Management()->Receive<common::msg::management::TrackingStoppedGrooveDataTimeout>());

      input(medium_conf_slice_data, std::chrono::milliseconds(1), false);
      CHECK(!fixture.Management()->Receive<common::msg::management::TrackingStoppedGrooveDataTimeout>());

      // Some additional input
      input(medium_conf_slice_data, no_confidence_grace, false);
      input(medium_conf_slice_data, no_confidence_grace, false);
      input(medium_conf_slice_data, no_confidence_grace, false);
      CHECK(!fixture.Management()->Receive<common::msg::management::TrackingStoppedGrooveDataTimeout>());
    }

    SUBCASE("First slice NO confidence ") {
      // Check that event is triggered and adaptio is stopped for scanner input with NO confidence level
      setup(weld_control::Mode::AUTOMATIC_BEAD_PLACEMENT);

      input(no_conf_slice_data, std::chrono::milliseconds(0), false);

      input(high_conf_slice_data, std::chrono::milliseconds(0), false);
      CHECK(!fixture.Management()->Receive<common::msg::management::TrackingStoppedGrooveDataTimeout>());

      /* move time forward and test again */
      input(high_conf_slice_data, 2 * no_confidence_grace, false);
      CHECK(!fixture.Management()->Receive<common::msg::management::TrackingStoppedGrooveDataTimeout>());
    }

    SUBCASE("LOW confidence - failed handover") {
      // Check LOW confidence handling - after handover notification confidence level is ignored.
      // The expected handling is that the mode changed to manual but in this test there is no mode
      // change and adaptio is stopped with a failed handover event.
      setup(weld_control::Mode::AUTOMATIC_BEAD_PLACEMENT);

      input(low_conf_slice_data, std::chrono::milliseconds(0), false);

      input(low_conf_slice_data, low_confidence_grace, false);
      CHECK(!fixture.Management()->Receive<common::msg::management::NotifyHandoverToManual>());

      input(low_conf_slice_data, std::chrono::milliseconds(1), false);
      CHECK(fixture.Management()->Receive<common::msg::management::NotifyHandoverToManual>());

      input(medium_conf_slice_data, handover_grace, false);
      CHECK(!fixture.Management()->Receive<common::msg::management::ScannerError>());

      input(high_conf_slice_data, std::chrono::milliseconds(1), true);
      CHECK(fixture.Management()->Receive<common::msg::management::ScannerError>());
      CheckEvents(fixture, {event_handover_failed});
    }

    SUBCASE("LOW confidence - cleared") {
      // Check LOW confidence handling is cleared when receiving a slice with MEDIUM confidence level
      setup(weld_control::Mode::JOINT_TRACKING);

      input(low_conf_slice_data, std::chrono::milliseconds(0), false);

      input(low_conf_slice_data, low_confidence_grace, false);
      CHECK(!fixture.Management()->Receive<common::msg::management::TrackingStoppedGrooveDataTimeout>());

      input(medium_conf_slice_data, std::chrono::milliseconds(1), false);
      CHECK(!fixture.Management()->Receive<common::msg::management::TrackingStoppedGrooveDataTimeout>());

      // Some additional input
      input(medium_conf_slice_data, no_confidence_grace, false);
      CHECK(!fixture.Management()->Receive<common::msg::management::TrackingStoppedGrooveDataTimeout>());
    }

    SUBCASE("NO confidence override LOW confidence") {
      // Check that NO confidence overrides LOW confidence
      setup(weld_control::Mode::JOINT_TRACKING);

      input(low_conf_slice_data, std::chrono::milliseconds(0), false);

      input(low_conf_slice_data, low_confidence_grace, false);
      CHECK(!fixture.Management()->Receive<common::msg::management::NotifyHandoverToManual>());

      input(low_conf_slice_data, std::chrono::milliseconds(1), false);
      CHECK(fixture.Management()->Receive<common::msg::management::NotifyHandoverToManual>());

      // override ongoing LOW confidence handover
      input(no_conf_slice_data, std::chrono::milliseconds(0), false);
      input(no_conf_slice_data, no_confidence_grace, false);
      CHECK(!fixture.Management()->Receive<common::msg::management::TrackingStoppedGrooveDataTimeout>());

      input(no_conf_slice_data, std::chrono::milliseconds(1), false);
      CHECK(fixture.Management()->Receive<common::msg::management::TrackingStoppedGrooveDataTimeout>());

      CheckEvents(fixture, {event_groove_detection_error});
    }
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access)
