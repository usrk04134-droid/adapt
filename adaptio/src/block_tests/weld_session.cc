#include <doctest/doctest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

// #define TESTLOG_DISABLED

#include "common/messages/kinematics.h"
#include "common/messages/management.h"
#include "common/messages/scanner.h"
#include "common/messages/weld_system.h"
#include "helpers/helpers.h"
#include "helpers/helpers_kinematics.h"
#include "helpers/helpers_settings.h"
#include "helpers/helpers_simulator.h"
#include "helpers/helpers_state_monitor.h"
#include "helpers/helpers_weld_control.h"
#include "helpers/helpers_weld_system.h"
#include "point3d.h"
#include "sim-config.h"
#include "simulator_interface.h"
#include "test_utils/testlog.h"
#include "torch-interfaces.h"
#include "tracking/tracking_manager.h"
#include "weld_system_client/weld_system_types.h"

//  NOLINTBEGIN(*-magic-numbers, *-optional-access)

namespace depsim   = deposition_simulator;
namespace help_sim = helpers_simulator;
using method       = weld_system::WeldSystemSettings::Method;

namespace {
//
// Constants
//
const int NUMBER_OF_STEPS_PER_REV{20};
const double DELTA_ANGLE{2. * help_sim::PI / NUMBER_OF_STEPS_PER_REV};
const int START_BEAD_STATE_MONITORING_AT_STEP{static_cast<int>(NUMBER_OF_STEPS_PER_REV * 0.9)};
const int CHECK_MONITORED_BEAD_STATE_AT_STEP{static_cast<int>(NUMBER_OF_STEPS_PER_REV * 0.8)};

const double SCANNER_MOUNT_ANGLE = 6.0 * help_sim::PI / 180.0;

struct TestContext {
  TestFixture fixture;
  std::unique_ptr<depsim::ISimulator> simulator;
  depsim::SimConfig sim_config;
  common::msg::weld_system::GetWeldSystemDataRsp weld_system_1_status_rsp;
  common::msg::weld_system::GetWeldSystemDataRsp weld_system_2_status_rsp;
  double ws1_wire_feed_speed_mm_per_s;
  double ws2_wire_feed_speed_mm_per_s;
  std::shared_ptr<depsim::ISingleWireTorch> depsim_ws1_torch;
  std::shared_ptr<depsim::ISingleWireTorch> depsim_ws2_torch;
  depsim::Point3d torch_pos_macs;
  std::chrono::milliseconds time_per_step_ms;

  // Test state that persists across RunTest calls
  bool test_ready = false;
  std::map<int, int> beads_per_layer;
  int current_layer                  = 1;
  double adaptive_weld_speed_m_per_s = 0.0;
  bool test_stop                     = false;
  int total_steps_executed           = 0;
  StateMonitor<std::string> state_monitor;
  double current_angle                    = 0.0;
  double current_lin_weld_object_distance = 0.0;
  int current_step_in_revolution          = 0;
};

auto StartJT(TestContext &context, help_sim::TestParameters &test_parameters) -> void {
  common::msg::management::TrackingStart start_joint_tracking_msg{
      .joint_tracking_mode = static_cast<uint32_t>(tracking::TrackingMode::TRACKING_LEFT_HEIGHT),
      .horizontal_offset   = 10.0,
      .vertical_offset     = static_cast<float>(help_sim::ConvertM2Mm(test_parameters.welding_parameters.stickout_m))};
  context.fixture.Management()->Dispatch(start_joint_tracking_msg);

  // Receive StartScanner
  REQUIRE_MESSAGE(context.fixture.Scanner()->Receive<common::msg::scanner::Start>(), "No Start msg received");

  CheckWeldControlStatus(context.fixture, WeldControlStatus{.weld_control_mode = "jt"});
}

auto StartABPWelding(TestContext &context) -> void {
  StartABP(context.fixture);
  CheckWeldControlStatus(context.fixture, WeldControlStatus{.weld_control_mode = "abp"});

  DispatchWeldSystemStateChange(context.fixture, weld_system::WeldSystemId::ID1,
                                common::msg::weld_system::OnWeldSystemStateChange::State::ARCING);
  DispatchWeldSystemStateChange(context.fixture, weld_system::WeldSystemId::ID2,
                                common::msg::weld_system::OnWeldSystemStateChange::State::ARCING);
}

auto SetupTest(help_sim::TestParameters &test_parameters) -> TestContext {
  TestContext context;

  context.time_per_step_ms = std::chrono::milliseconds{static_cast<int>(help_sim::CalculateStepTimeMs(
      test_parameters.welding_parameters.weld_object_diameter_m,
      test_parameters.welding_parameters.weld_object_speed_cm_per_min, NUMBER_OF_STEPS_PER_REV))};

  // WS1
  context.weld_system_1_status_rsp = {
      .voltage = static_cast<float>(test_parameters.welding_parameters.weld_system_1.voltage),
      .current = static_cast<float>(test_parameters.welding_parameters.weld_system_1.current),
      .wire_lin_velocity =
          static_cast<float>(test_parameters.welding_parameters.weld_system_1.wire_lin_velocity_mm_per_sec),
      .deposition_rate = static_cast<float>(test_parameters.welding_parameters.weld_system_1.deposition_rate),
      .heat_input      = static_cast<float>(test_parameters.welding_parameters.weld_system_1.heat_input),
      .twin_wire       = test_parameters.welding_parameters.weld_system_1.twin_wire,
      .wire_diameter   = static_cast<float>(test_parameters.welding_parameters.weld_system_1.wire_diameter_mm),
  };
  // WS2
  context.weld_system_2_status_rsp = {
      .voltage = static_cast<float>(test_parameters.welding_parameters.weld_system_2.voltage),
      .current = static_cast<float>(test_parameters.welding_parameters.weld_system_2.current),
      .wire_lin_velocity =
          static_cast<float>(test_parameters.welding_parameters.weld_system_2.wire_lin_velocity_mm_per_sec),
      .deposition_rate = static_cast<float>(test_parameters.welding_parameters.weld_system_2.deposition_rate),
      .heat_input      = static_cast<float>(test_parameters.welding_parameters.weld_system_2.heat_input),
      .twin_wire       = test_parameters.welding_parameters.weld_system_2.twin_wire,
      .wire_diameter   = static_cast<float>(test_parameters.welding_parameters.weld_system_2.wire_diameter_mm),
  };

  auto weld_control_config                   = context.fixture.GetConfigManagerMock()->GetWeldControlConfiguration();
  weld_control_config.scanner_input_interval = std::chrono::milliseconds{1884};
  weld_control_config.adaptivity.gaussian_filter.kernel_size = 9;
  weld_control_config.adaptivity.gaussian_filter.sigma       = 3.0;
  weld_control_config.handover_grace                         = std::chrono::seconds{25};
  weld_control_config.storage_resolution =
      test_parameters.welding_parameters.weld_object_diameter_m * std::numbers::pi * 1000.0 / NUMBER_OF_STEPS_PER_REV;
  context.fixture.GetConfigManagerMock()->SetWeldControlConfiguration(weld_control_config);

  context.fixture.GetConfigManagerMock()->SetWeldControlConfiguration(weld_control_config);

  context.fixture.StartApplication();

  StoreSettings(context.fixture, TestSettings{.use_edge_sensor = test_parameters.welding_parameters.use_edge_sensor},
                true);

  // Set up to use timer wrapper
  context.fixture.SetupTimerWrapper();

  // Adaptive variables
  context.ws1_wire_feed_speed_mm_per_s = helpers_simulator::CalculateWireSpeedMmPerSec(
      method::DC, test_parameters.welding_parameters.weld_system_1.wire_diameter_mm,
      context.weld_system_1_status_rsp.current);
  context.ws2_wire_feed_speed_mm_per_s = helpers_simulator::CalculateWireSpeedMmPerSec(
      method::AC, test_parameters.welding_parameters.weld_system_2.wire_diameter_mm,
      context.weld_system_2_status_rsp.current);

  // Create simulator
  context.simulator  = depsim::CreateSimulator();
  context.sim_config = context.simulator->CreateSimConfig();
  help_sim::SetSimulatorDefault(context.sim_config, NUMBER_OF_STEPS_PER_REV);

  // Set up joint geometry
  help_sim::SetJointGeometry(context.fixture, context.sim_config, test_parameters.test_joint_geometry);

  context.sim_config.travel_speed =
      help_sim::ConvertCMPerMin2MPerS(test_parameters.welding_parameters.weld_object_speed_cm_per_min);

  // Add adaptive torches
  context.depsim_ws1_torch = context.simulator->AddSingleWireTorch(
      help_sim::ConvertMm2M(test_parameters.welding_parameters.weld_system_1.wire_diameter_mm),
      help_sim::ConvertMmPerS2MPerS(context.ws1_wire_feed_speed_mm_per_s));
  context.depsim_ws2_torch = context.simulator->AddSingleWireTorch(
      help_sim::ConvertMm2M(test_parameters.welding_parameters.weld_system_2.wire_diameter_mm),
      help_sim::ConvertMmPerS2MPerS(context.ws2_wire_feed_speed_mm_per_s));

  // Simulator config OPCS/LPCS
  help_sim::ConfigOPCS(context.sim_config, test_parameters.welding_parameters.weld_object_diameter_m,
                       test_parameters.welding_parameters.stickout_m);
  help_sim::ConfigLPCS(context.sim_config, test_parameters.welding_parameters.stickout_m, SCANNER_MOUNT_ANGLE);

  context.simulator->Initialize(context.sim_config);

  DispatchKinematicsStateChange(context.fixture, common::msg::kinematics::StateChange::State::HOMED);
  DispatchKinematicsEdgeStateChange(context.fixture, common::msg::kinematics::EdgeStateChange::State::AVAILABLE);

  context.fixture.Management()->Dispatch(common::msg::management::SubscribeReadyState{});
  auto ready_msg = context.fixture.Management()->Receive<common::msg::management::ReadyState>();
  CHECK(ready_msg.has_value());
  CHECK_EQ(ready_msg->state, common::msg::management::ReadyState::State::TRACKING_READY);

  context.simulator->Initialize(context.sim_config);

  context.torch_pos_macs =
      depsim::Point3d(-30e-3, 0, -19e-3 + test_parameters.welding_parameters.stickout_m, depsim::MACS);
  context.simulator->UpdateTorchPosition(context.torch_pos_macs);
  TESTLOG("Set DepSim torch start position MACS, vertical {}m, horizontal: {}m", context.torch_pos_macs.GetX(),
          context.torch_pos_macs.GetZ());

  //
  // set up adaptio
  //
  auto abws        = helpers_simulator::ConvertFromOptionalAbwVector(context.simulator->GetAbwPoints(depsim::MACS));
  auto joint_depth = std::abs(abws[0].GetZ() - abws[1].GetZ());  // Approximate depth
  for (auto &p : abws) {
    joint_depth = std::max(std::abs(abws[0].GetZ() - p.GetZ()), joint_depth);
  }

  context.simulator->UpdateTorchPosition(context.torch_pos_macs);
  TESTLOG("Set DepSim torch position MACS (1st fill layer), horizontal {:.5f}m, vertical: {:.5f}m",
          context.torch_pos_macs.GetX(), context.torch_pos_macs.GetZ());

  help_sim::SetABPParameters(context.fixture, test_parameters);

  CheckAndDispatchGetWeldAxis(context.fixture, 0.0, 0.0, 0.0,
                              help_sim::ConvertM2Mm(context.sim_config.joint_def_left.outer_diameter / 2.));

  // Initialize test state
  context.adaptive_weld_speed_m_per_s = context.sim_config.travel_speed;

  return context;
}

auto RunTest(TestContext &context, help_sim::TestParameters &test_parameters, int max_steps = -1,
             bool stop_on_ready_for_cap = false) -> void {
  TESTLOG("-= RunTest max_steps={} =-", max_steps);

  // Reset the cap ready stop flag to allow resuming
  int steps_executed_this_call = 0;

  while (!context.test_ready) {
    auto abws        = helpers_simulator::ConvertFromOptionalAbwVector(context.simulator->GetAbwPoints(depsim::MACS));
    auto joint_depth = std::abs(abws[0].GetZ() - abws[1].GetZ());  // Approximate depth
    for (auto &p : abws) {
      joint_depth = std::max(std::abs(abws[0].GetZ() - p.GetZ()), joint_depth);
    }

    for (int step = context.current_step_in_revolution; step < NUMBER_OF_STEPS_PER_REV; step++) {
      // Check if we've reached the maximum number of steps for this call
      if (max_steps > 0 && steps_executed_this_call >= max_steps) {
        context.current_step_in_revolution = step;
        TESTLOG("Reached maximum step limit of {} for this call, stopping test (can be resumed)", max_steps);
        return;
      }

      context.total_steps_executed++;
      steps_executed_this_call++;

      abws = helpers_simulator::ConvertFromOptionalAbwVector(context.simulator->GetAbwPoints(depsim::LPCS));
      //
      // Update adaptio
      //
      // ABW points on scanner interface
      auto slice_data = helpers_simulator::GetSliceData(
          abws, *context.simulator,
          static_cast<std::uint64_t>(
              context.fixture.GetClockNowFuncWrapper()->GetSystemClock().time_since_epoch().count()));
      context.fixture.Scanner()->Dispatch(slice_data);

      auto get_slides_position = context.fixture.Kinematics()->Receive<common::msg::kinematics::GetSlidesPosition>();
      context.fixture.Kinematics()->Dispatch(common::msg::kinematics::GetSlidesPositionRsp{
          .client_id  = get_slides_position->client_id,
          .time_stamp = get_slides_position->time_stamp,
          .horizontal = help_sim::ConvertM2Mm(context.torch_pos_macs.GetX()),
          .vertical   = help_sim::ConvertM2Mm(context.torch_pos_macs.GetZ())});

      auto get_edge_position = context.fixture.Kinematics()->Receive<common::msg::kinematics::GetEdgePosition>();
      context.fixture.Kinematics()->Dispatch(
          common::msg::kinematics::GetEdgePositionRsp{.client_id = get_edge_position->client_id, .position = 0.0});

      //  Check GetWeldAxis request and dispatch response
      CheckAndDispatchGetWeldAxis(context.fixture, context.current_angle, context.current_lin_weld_object_distance,
                                  help_sim::ConvertMPerS2RadPerS(context.adaptive_weld_speed_m_per_s,
                                                                 context.sim_config.joint_def_left.outer_diameter / 2.),
                                  help_sim::ConvertM2Mm(context.sim_config.joint_def_left.outer_diameter / 2.));

      // Check GetWeldSystemStatus requests for both weld-systems and send response
      context.weld_system_1_status_rsp.heat_input = static_cast<float>(help_sim::CalculateHeatInputValue(
          context.weld_system_1_status_rsp.voltage, context.weld_system_1_status_rsp.current,
          context.adaptive_weld_speed_m_per_s));
      context.weld_system_2_status_rsp.heat_input = static_cast<float>(help_sim::CalculateHeatInputValue(
          context.weld_system_2_status_rsp.voltage, context.weld_system_2_status_rsp.current,
          context.adaptive_weld_speed_m_per_s));
      CheckAndDispatchWeldSystemDataRsp(context.fixture, weld_system::WeldSystemId::ID1,
                                        context.weld_system_1_status_rsp);
      CheckAndDispatchWeldSystemDataRsp(context.fixture, weld_system::WeldSystemId::ID2,
                                        context.weld_system_2_status_rsp);

      // Recalculate adaptive variables, then update the simulator and the adaptio responses
      auto set_weld_axis = context.fixture.Kinematics()->Receive<common::msg::kinematics::SetWeldAxisData>();
      if (set_weld_axis) {
        context.adaptive_weld_speed_m_per_s = help_sim::ConvertRadPerS2MPerS(
            set_weld_axis->velocity, context.sim_config.joint_def_left.outer_diameter / 2.);
        REQUIRE(context.adaptive_weld_speed_m_per_s >=
                doctest::Approx(help_sim::ConvertCMPerMin2MPerS(test_parameters.abp_parameters.weld_speed.min)));
        REQUIRE(context.adaptive_weld_speed_m_per_s <=
                help_sim::ConvertCMPerMin2MPerS(test_parameters.abp_parameters.weld_speed.max));
        context.simulator->UpdateTravelSpeed(context.adaptive_weld_speed_m_per_s);
      }
      auto set_welding_system_settings =
          context.fixture.WeldSystem()->Receive<common::msg::weld_system::SetWeldSystemSettings>();
      if (set_welding_system_settings) {
        REQUIRE(set_welding_system_settings->current >= test_parameters.abp_parameters.weld_system_2_current.min);
        REQUIRE(set_welding_system_settings->current <= test_parameters.abp_parameters.weld_system_2_current.max);
        context.ws1_wire_feed_speed_mm_per_s = help_sim::CalculateWireSpeedMmPerSec(
            method::DC, test_parameters.welding_parameters.weld_system_1.wire_diameter_mm,
            context.weld_system_1_status_rsp.current);
        context.ws2_wire_feed_speed_mm_per_s = help_sim::CalculateWireSpeedMmPerSec(
            method::AC, test_parameters.welding_parameters.weld_system_2.wire_diameter_mm,
            set_welding_system_settings->current);
        context.depsim_ws1_torch->SetWireFeedSpeed(help_sim::ConvertMmPerS2MPerS(context.ws1_wire_feed_speed_mm_per_s));
        context.depsim_ws2_torch->SetWireFeedSpeed(help_sim::ConvertMmPerS2MPerS(context.ws2_wire_feed_speed_mm_per_s));
        context.weld_system_1_status_rsp.wire_lin_velocity = static_cast<float>(context.ws1_wire_feed_speed_mm_per_s);
        context.weld_system_2_status_rsp.wire_lin_velocity = static_cast<float>(context.ws2_wire_feed_speed_mm_per_s);
        context.weld_system_2_status_rsp.current           = set_welding_system_settings->current;
      }
      // Dispatch expired timers
      context.fixture.GetTimerWrapper()->DispatchAllExpired();

      //
      // Check adaptio output
      //

      if (test_parameters.welding_parameters.use_edge_sensor &&
          context.fixture.Management()->Receive<common::msg::management::GracefulStop>()) {
        TESTLOG("Received GracefulStop - exit test!");
        context.test_stop = true;

      } else if (!test_parameters.welding_parameters.use_edge_sensor &&
                 context.fixture.Management()->Receive<common::msg::management::NotifyHandoverToManual>()) {
        TESTLOG("Received NotifyHandoverToManual - exit test!");
        context.test_stop = true;
      } else {
        // Receive SetSlidesPosition and update DepSim torch position
        auto set_slides_position = context.fixture.Kinematics()->Receive<common::msg::kinematics::SetSlidesPosition>();
        if (set_slides_position) {
          context.torch_pos_macs = depsim::Point3d(help_sim::ConvertMm2M(set_slides_position->horizontal), 0,
                                                   help_sim::ConvertMm2M(set_slides_position->vertical), depsim::MACS);
          context.simulator->UpdateTorchPosition(context.torch_pos_macs);
        } else {
          // When resuming, the message might not be available yet - use current position
          TESTLOG("No SetSlidesPosition received (likely resuming mid-revolution), using current position");
        }
      }

      {  // Check ABP weld control status
        auto weld_control_status = GetWeldControlStatus(context.fixture);
        context.current_layer    = weld_control_status.layer_number.value();
        // Check if last layer completed
        if (context.test_stop) {
          context.test_ready = true;
          break;
        }
        if (weld_control_status.weld_control_mode.has_value()) {
          CHECK_EQ(weld_control_status.weld_control_mode.value(), "abp");
        }
        // Check bead control state by monitoring state changes
        if (step == START_BEAD_STATE_MONITORING_AT_STEP) {
          context.state_monitor.Clear();
        }
        if (step > START_BEAD_STATE_MONITORING_AT_STEP || step < CHECK_MONITORED_BEAD_STATE_AT_STEP) {
          context.state_monitor.Add(weld_control_status.bead_operation.value());
        }

        context.beads_per_layer[weld_control_status.layer_number.value()] = weld_control_status.bead_number.value();
      }

      auto const msg = context.fixture.Management()->Receive<common::msg::management::ReadyState>();
      if (msg) {
        switch (msg->state) {
          case common::msg::management::ReadyState::State::NOT_READY:
          case common::msg::management::ReadyState::State::NOT_READY_AUTO_CAL_MOVE:
          case common::msg::management::ReadyState::State::TRACKING_READY:
          case common::msg::management::ReadyState::State::ABP_READY:
            break;
          case common::msg::management::ReadyState::State::ABP_CAP_READY:
          case common::msg::management::ReadyState::State::ABP_AND_ABP_CAP_READY:
            if (stop_on_ready_for_cap) {
              TESTLOG("Received ABP_CAP_READY or ABP_AND_ABP_CAP_READY - stopping test!");
              context.test_stop = true;
            } else {
              StartABPCap(context.fixture);
            }
            break;
        }
      }

      // Step position
      context.current_angle += DELTA_ANGLE;
      context.current_lin_weld_object_distance +=
          help_sim::ConvertM2Mm(context.sim_config.joint_def_left.outer_diameter / 2.) * DELTA_ANGLE;
      context.current_angle = std::fmod(context.current_angle, 2.0 * help_sim::PI);
      context.simulator->RunWithRotation(DELTA_ANGLE,
                                         test_parameters.test_joint_geometry.simulator_joint_geometry.bead_radians_m);
      // Step clocks
      context.fixture.GetClockNowFuncWrapper()->StepSystemClock(context.time_per_step_ms);
      context.fixture.GetClockNowFuncWrapper()->StepSteadyClock(context.time_per_step_ms);
    }

    // Reset step counter for next revolution
    context.current_step_in_revolution = 0;

    // Break out of outer loop if test is ready
    if (context.test_ready) {
      break;
    }
  }

  // Only check final results if test completed naturally (not stopped due to step limit)
  if (context.test_ready) {
    std::ostringstream error_string;
    error_string << "Unexpected number of beads layer\nactual:   ";
    for (auto [layer, beads] : context.beads_per_layer) {
      error_string << "{" << layer << ", " << beads << "} ";
    }

    error_string << "\nexpected: ";
    for (auto [layer, beads] : test_parameters.testcase_parameters.expected_beads_in_layer) {
      error_string << "{" << layer << ", " << beads << "} ";
    }

    REQUIRE_MESSAGE(context.beads_per_layer == test_parameters.testcase_parameters.expected_beads_in_layer,
                    error_string.str());
  } else {
    TESTLOG("Test paused after {} total steps, {} steps this call - can be resumed", context.total_steps_executed,
            steps_executed_this_call);
  }
}
}  // namespace

TEST_SUITE("ABPResume") {
  TEST_CASE("PauseAndResumeFill") {
    help_sim::TestParameters test_parameters{
        .abp_parameters{.wall_offset_mm = 2.,
                        .bead_overlap   = 20.,
                        .step_up_value  = 0.5,
                        .k_gain         = 2.,
                        .heat_input{.min = 2.1, .max = 3.4},
                        .weld_system_2_current{.min = 700., .max = 800.},
                        .weld_speed{.min = 80., .max = 95.},
                        .bead_switch_angle = 15.,
                        .cap_corner_offset = 5.0,
                        .cap_beads         = 6,
                        .cap_init_depth    = 7.0},
        .welding_parameters{
                        .weld_object_diameter_m       = 2.,
                        .weld_object_speed_cm_per_min = 100.,
                        .stickout_m                   = 25e-3,
                        .weld_system_1{.voltage                      = 29.0,
                           .current                      = 700.0,
                           .wire_lin_velocity_mm_per_sec = help_sim::CalculateWireSpeedMmPerSec(method::DC, 4, 700),
                           .deposition_rate              = 10.4,
                           .heat_input                   = 1.2,
                           .twin_wire                    = false,
                           .wire_diameter_mm             = 4.0},
                        .weld_system_2{.voltage                      = 31.0,
                           .current                      = 700.0,
                           .wire_lin_velocity_mm_per_sec = help_sim::CalculateWireSpeedMmPerSec(method::AC, 4, 700),
                           .deposition_rate              = 10.6,
                           .heat_input                   = 1.2,
                           .twin_wire                    = false,
                           .wire_diameter_mm             = 4.0},
                        .use_edge_sensor = true},
        .test_joint_geometry{help_sim::TEST_JOINT_GEOMETRY_WIDE},
        .testcase_parameters{.expected_beads_in_layer{{1, 3}, {2, 4}, {3, 4}, {4, 5}, {5, 6}}}
    };
    auto context = SetupTest(test_parameters);
    StartJT(context, test_parameters);
    StartABPWelding(context);

    // Run 1 full bead + ~50% of the second bead in the first layer being welded
    RunTest(context, test_parameters, NUMBER_OF_STEPS_PER_REV * 1.5);

    auto status = GetWeldControlStatus(context.fixture);
    CHECK(*status.weld_control_mode == "abp");
    CHECK(*status.bead_operation == "steady");
    CHECK(*status.layer_number == 1);
    CHECK(*status.bead_number == 2);
    CHECK(*status.progress == doctest::Approx(0.2).epsilon(0.1));

    // 1st Stop and change weld-system state
    context.fixture.Management()->Dispatch(common::msg::management::Stop());
    DispatchWeldSystemStateChange(context.fixture, weld_system::WeldSystemId::ID1,
                                  common::msg::weld_system::OnWeldSystemStateChange::State::READY_TO_START);
    DispatchWeldSystemStateChange(context.fixture, weld_system::WeldSystemId::ID2,
                                  common::msg::weld_system::OnWeldSystemStateChange::State::READY_TO_START);

    // Start ABP again and see that it starts with cleared weld session data
    StartJT(context, test_parameters);
    StartABPWelding(context);
    RunTest(context, test_parameters, NUMBER_OF_STEPS_PER_REV * 0.5);

    status = GetWeldControlStatus(context.fixture);
    CHECK(*status.weld_control_mode == "abp");
    CHECK(*status.bead_operation == "steady");
    CHECK(*status.layer_number == 1);
    CHECK(*status.bead_number == 2);
    CHECK(*status.progress == doctest::Approx(0.7).epsilon(0.1));

    // 2nd Stop and change weld-system state
    context.fixture.Management()->Dispatch(common::msg::management::Stop());
    DispatchWeldSystemStateChange(context.fixture, weld_system::WeldSystemId::ID1,
                                  common::msg::weld_system::OnWeldSystemStateChange::State::READY_TO_START);
    DispatchWeldSystemStateChange(context.fixture, weld_system::WeldSystemId::ID2,
                                  common::msg::weld_system::OnWeldSystemStateChange::State::READY_TO_START);

    // Start ABP again and see that it starts with cleared weld session data
    StartJT(context, test_parameters);
    StartABPWelding(context);
    RunTest(context, test_parameters, NUMBER_OF_STEPS_PER_REV * 2.0);

    status = GetWeldControlStatus(context.fixture);
    CHECK(*status.weld_control_mode == "abp");
    CHECK(*status.bead_operation == "steady");
    CHECK(*status.layer_number == 2);
    CHECK(*status.bead_number == 1);
    CHECK(*status.progress == doctest::Approx(0.5).epsilon(0.1));

    // Run to completion
    RunTest(context, test_parameters);
  }

  TEST_CASE("ClearSessionAndResume") {
    help_sim::TestParameters test_parameters{
        .abp_parameters{.wall_offset_mm = 2.,
                        .bead_overlap   = 20.,
                        .step_up_value  = 0.5,
                        .k_gain         = 2.,
                        .heat_input{.min = 2.1, .max = 3.4},
                        .weld_system_2_current{.min = 700., .max = 800.},
                        .weld_speed{.min = 80., .max = 95.},
                        .bead_switch_angle = 15.,
                        .cap_corner_offset = 5.0,
                        .cap_beads         = 6,
                        .cap_init_depth    = 7.0},
        .welding_parameters{
                        .weld_object_diameter_m       = 2.,
                        .weld_object_speed_cm_per_min = 100.,
                        .stickout_m                   = 25e-3,
                        .weld_system_1{.voltage                      = 29.0,
                           .current                      = 700.0,
                           .wire_lin_velocity_mm_per_sec = help_sim::CalculateWireSpeedMmPerSec(method::DC, 4, 700),
                           .deposition_rate              = 10.4,
                           .heat_input                   = 1.2,
                           .twin_wire                    = false,
                           .wire_diameter_mm             = 4.0},
                        .weld_system_2{.voltage                      = 31.0,
                           .current                      = 700.0,
                           .wire_lin_velocity_mm_per_sec = help_sim::CalculateWireSpeedMmPerSec(method::AC, 4, 700),
                           .deposition_rate              = 10.6,
                           .heat_input                   = 1.2,
                           .twin_wire                    = false,
                           .wire_diameter_mm             = 4.0},
                        .use_edge_sensor = true},
        .test_joint_geometry{help_sim::TEST_JOINT_GEOMETRY_WIDE},
    };
    auto context = SetupTest(test_parameters);
    StartJT(context, test_parameters);
    StartABPWelding(context);

    // Run 1 full bead + ~50% of the second bead in the first layer being welded
    RunTest(context, test_parameters, NUMBER_OF_STEPS_PER_REV * 1.5);

    auto const status1 = GetWeldControlStatus(context.fixture);
    CHECK(*status1.weld_control_mode == "abp");
    CHECK(*status1.bead_operation == "steady");
    CHECK(*status1.bead_number == 2);
    CHECK(*status1.progress == doctest::Approx(0.2).epsilon(0.1));

    // Check that clearing the weld session with ABP active is not allowed
    ClearWeldSessionAndCheckResponse(context.fixture, false);

    // Check that weld-control status has not changed
    auto status2 = GetWeldControlStatus(context.fixture);
    CHECK(*status1.weld_control_mode == *status2.weld_control_mode);
    CHECK(*status1.bead_operation == *status2.bead_operation);
    CHECK(*status1.bead_number == *status2.bead_number);
    CHECK(*status1.progress == doctest::Approx(*status2.progress).epsilon(0.0001));

    // Stop and change weld-system state
    context.fixture.Management()->Dispatch(common::msg::management::Stop());
    DispatchWeldSystemStateChange(context.fixture, weld_system::WeldSystemId::ID1,
                                  common::msg::weld_system::OnWeldSystemStateChange::State::READY_TO_START);
    DispatchWeldSystemStateChange(context.fixture, weld_system::WeldSystemId::ID2,
                                  common::msg::weld_system::OnWeldSystemStateChange::State::READY_TO_START);

    // Check that weld-control status is not changed after Stop
    status2 = GetWeldControlStatus(context.fixture);
    CHECK(*status2.weld_control_mode == "idle");
    CHECK(*status1.bead_operation == *status2.bead_operation);
    CHECK(*status1.bead_number == *status2.bead_number);
    CHECK(*status1.progress == doctest::Approx(*status2.progress).epsilon(0.0001));

    // Check that it is possible to clear the weld session when ABP is not active.
    ClearWeldSessionAndCheckResponse(context.fixture, true);

    status2 = GetWeldControlStatus(context.fixture);
    CHECK(*status2.weld_control_mode == "idle");
    CHECK(*status2.bead_operation == "idle");
    CHECK(*status2.bead_number == 0);
    CHECK(*status2.progress == doctest::Approx(0.0).epsilon(0.0001));

    // Start ABP again and see that it starts with cleared weld session data
    StartJT(context, test_parameters);
    StartABPWelding(context);
    RunTest(context, test_parameters, NUMBER_OF_STEPS_PER_REV * 0.5);

    status2 = GetWeldControlStatus(context.fixture);
    CHECK(*status2.weld_control_mode == "abp");
    CHECK(*status2.bead_operation == "steady");
    CHECK(*status2.bead_number == 1);
    CHECK(*status2.progress == doctest::Approx(0.4).epsilon(0.1));
  }

  TEST_CASE("ClearSessionNonABPWelding") {
    help_sim::TestParameters test_parameters{
        .abp_parameters{.wall_offset_mm = 2.,
                        .bead_overlap   = 20.,
                        .step_up_value  = 0.5,
                        .k_gain         = 2.,
                        .heat_input{.min = 2.1, .max = 3.4},
                        .weld_system_2_current{.min = 700., .max = 800.},
                        .weld_speed{.min = 80., .max = 95.},
                        .bead_switch_angle = 15.,
                        .cap_corner_offset = 5.0,
                        .cap_beads         = 6,
                        .cap_init_depth    = 7.0},
        .welding_parameters{
                        .weld_object_diameter_m       = 2.,
                        .weld_object_speed_cm_per_min = 100.,
                        .stickout_m                   = 25e-3,
                        .weld_system_1{.voltage                      = 29.0,
                           .current                      = 700.0,
                           .wire_lin_velocity_mm_per_sec = help_sim::CalculateWireSpeedMmPerSec(method::DC, 4, 700),
                           .deposition_rate              = 10.4,
                           .heat_input                   = 1.2,
                           .twin_wire                    = false,
                           .wire_diameter_mm             = 4.0},
                        .weld_system_2{.voltage                      = 31.0,
                           .current                      = 700.0,
                           .wire_lin_velocity_mm_per_sec = help_sim::CalculateWireSpeedMmPerSec(method::AC, 4, 700),
                           .deposition_rate              = 10.6,
                           .heat_input                   = 1.2,
                           .twin_wire                    = false,
                           .wire_diameter_mm             = 4.0},
                        .use_edge_sensor = true},
        .test_joint_geometry{help_sim::TEST_JOINT_GEOMETRY_WIDE},
    };
    auto context = SetupTest(test_parameters);
    StartJT(context, test_parameters);
    StartABPWelding(context);

    // Run 1 full bead + ~50% of the second bead in the first layer being welded
    RunTest(context, test_parameters, NUMBER_OF_STEPS_PER_REV * 1.5);

    auto const status1 = GetWeldControlStatus(context.fixture);
    CHECK(*status1.weld_control_mode == "abp");
    CHECK(*status1.bead_operation == "steady");
    CHECK(*status1.bead_number == 2);
    CHECK(*status1.progress == doctest::Approx(0.2).epsilon(0.1));

    // Stop and change weld-system state
    context.fixture.Management()->Dispatch(common::msg::management::Stop());
    DispatchWeldSystemStateChange(context.fixture, weld_system::WeldSystemId::ID1,
                                  common::msg::weld_system::OnWeldSystemStateChange::State::READY_TO_START);
    DispatchWeldSystemStateChange(context.fixture, weld_system::WeldSystemId::ID2,
                                  common::msg::weld_system::OnWeldSystemStateChange::State::READY_TO_START);

    StartJT(context, test_parameters);

    // Check that weld-control status has not changed by activating JT
    auto status2 = GetWeldControlStatus(context.fixture);
    CHECK(*status2.weld_control_mode == "jt");
    CHECK(*status1.bead_operation == *status2.bead_operation);
    CHECK(*status1.bead_number == *status2.bead_number);
    CHECK(*status1.progress == doctest::Approx(*status2.progress).epsilon(0.0001));

    // 1. Manual welding without ABP is not allowed
    DispatchWeldSystemStateChange(context.fixture, weld_system::WeldSystemId::ID1,
                                  common::msg::weld_system::OnWeldSystemStateChange::State::ARCING);
    DispatchWeldSystemStateChange(context.fixture, weld_system::WeldSystemId::ID2,
                                  common::msg::weld_system::OnWeldSystemStateChange::State::ARCING);

    status2 = GetWeldControlStatus(context.fixture);
    CHECK(*status2.weld_control_mode == "jt");
    CHECK(*status2.bead_operation == "idle");
    CHECK(*status2.bead_number == 0);
    CHECK(*status2.progress == doctest::Approx(0.0).epsilon(0.0001));

    // Start ABP and weld parts of the first bead (new weld session)
    StartABPWelding(context);
    RunTest(context, test_parameters, NUMBER_OF_STEPS_PER_REV * 0.5);

    status2 = GetWeldControlStatus(context.fixture);
    CHECK(*status2.weld_control_mode == "abp");
    CHECK(*status2.bead_operation == "steady");
    CHECK(*status2.bead_number == 1);
    CHECK(*status2.progress == doctest::Approx(0.4).epsilon(0.1));

    // 2. Stopping ABP when welding not allowed
    context.fixture.Management()->Dispatch(common::msg::management::ABPStop());

    status2 = GetWeldControlStatus(context.fixture);
    CHECK(*status2.weld_control_mode == "jt");
    CHECK(*status2.bead_operation == "idle");
    CHECK(*status2.bead_number == 0);
    CHECK(*status2.progress == doctest::Approx(0.0).epsilon(0.0001));
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access)
