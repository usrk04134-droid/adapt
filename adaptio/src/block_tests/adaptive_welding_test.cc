#include <doctest/doctest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <format>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#define TESTLOG_DISABLED

#include "block_tests/helpers_settings.h"
#include "block_tests/helpers_simulator.h"
#include "block_tests/helpers_weld_system.h"
#include "common/messages/kinematics.h"
#include "common/messages/management.h"
#include "common/messages/scanner.h"
#include "common/messages/weld_system.h"
#include "helpers.h"
#include "helpers_calibration_v2.h"
#include "helpers_kinematics.h"
#include "helpers_simulator.h"
#include "helpers_state_monitor.h"
#include "helpers_weld_control.h"
#include "helpers_weld_system.h"
#include "point3d.h"
#include "sim-config.h"
#include "simulator_interface.h"
#include "test_utils/testlog.h"
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
const int NUMBER_OF_STEPS_PER_REV{200};
const double DELTA_ANGLE{2. * help_sim::PI / NUMBER_OF_STEPS_PER_REV};
const int START_BEAD_STATE_MONITORING_AT_STEP{static_cast<int>(NUMBER_OF_STEPS_PER_REV * 0.9)};
const int CHECK_MONITORED_BEAD_STATE_AT_STEP{static_cast<int>(NUMBER_OF_STEPS_PER_REV * 0.8)};

const double SCANNER_MOUNT_ANGLE = 6.0 * help_sim::PI / 180.0;
const double TOUCH_POINT_DEPTH_M = 10e-3;

// Test function
auto AdativeWeldingTest(help_sim::TestParameters &test_parameters) -> void;
}  // namespace

TEST_SUITE("AdaptiveWeldingTest") {
  TEST_CASE("adaptive_welding_wide_joint_test") {
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

        .testcase_parameters{.expected_beads_in_layer{{1, 3}, {2, 4}, {3, 5}, {4, 5}, {5, 6}}}
    };
    AdativeWeldingTest(test_parameters);
  }

  TEST_CASE("adaptive_welding_misaligned_joint_test") {
    help_sim::TestParameters test_parameters{
        .abp_parameters{.wall_offset_mm = 2.,
                        .bead_overlap   = 20.,
                        .step_up_value  = 0.5,
                        .k_gain         = 2.,
                        .heat_input{.min = 2.1, .max = 3.4},
                        .weld_system_2_current{.min = 700., .max = 800.},
                        .weld_speed{.min = 80., .max = 95.},
                        .bead_switch_angle = 15.,
                        .cap_corner_offset = 2.0,
                        .cap_beads         = 5,
                        .cap_init_depth    = 7.0},
        .welding_parameters{
                        .weld_object_diameter_m       = 2.,
                        .weld_object_speed_cm_per_min = 100.,
                        .stickout_m                   = 25e-3,
                        .weld_system_1{.voltage                      = 29.0,
                           .current                      = 700.0,
                           .wire_lin_velocity_mm_per_sec = help_sim::CalculateWireSpeedMmPerSec(method::DC, 4., 700),
                           .deposition_rate              = 10.4,
                           .heat_input                   = 1.2,
                           .twin_wire                    = false,
                           .wire_diameter_mm             = 4.0},
                        .weld_system_2{.voltage                      = 31.0,
                           .current                      = 700.0,
                           .wire_lin_velocity_mm_per_sec = help_sim::CalculateWireSpeedMmPerSec(method::AC, 4., 700),
                           .deposition_rate              = 10.6,
                           .heat_input                   = 1.2,
                           .twin_wire                    = false,
                           .wire_diameter_mm             = 4.0},
                        .use_edge_sensor = true},
        .test_joint_geometry{help_sim::TEST_JOINT_GEOMETRY_WIDE},

        .testcase_parameters{.expected_beads_in_layer{{1, 3}, {2, 4}, {3, 4}, {4, 5}, {5, 5}}}  // Number of test layers
    };
    // Set a misaligned joint
    test_parameters.test_joint_geometry.simulator_joint_geometry.left.radial_offset_m       = 2e-3;
    test_parameters.test_joint_geometry.simulator_joint_geometry.right.radial_offset_m      = -2e-3;
    test_parameters.test_joint_geometry.simulator_joint_geometry.joint_depth_percentage     = 50;
    test_parameters.test_joint_geometry.simulator_joint_geometry.joint_bottom_curv_radius_m = 0.6;
    AdativeWeldingTest(test_parameters);
  }

  TEST_CASE("adaptive_welding_deep_joint") {
    help_sim::TestParameters test_parameters{
        .abp_parameters{.wall_offset_mm = 3.0,
                        .bead_overlap   = 20.,
                        .step_up_value  = 0.5,
                        .k_gain         = 2.,
                        .heat_input{.min = 2.1, .max = 3.4},
                        .weld_system_2_current{.min = 700., .max = 800.},
                        .weld_speed{.min = 80., .max = 95.},
                        .bead_switch_angle = 15.,
                        .cap_corner_offset = 0.0,
                        .cap_beads         = 6,
                        .cap_init_depth    = 7.0},
        .welding_parameters{
                        .weld_object_diameter_m       = 2.,
                        .weld_object_speed_cm_per_min = 100.,
                        .stickout_m                   = 25e-3,
                        .weld_system_1{.voltage                      = 29.0,
                           .current                      = 700.0,
                           .wire_lin_velocity_mm_per_sec = help_sim::CalculateWireSpeedMmPerSec(method::DC, 4., 700),
                           .deposition_rate              = 10.4,
                           .heat_input                   = 1.2,
                           .twin_wire                    = false,
                           .wire_diameter_mm             = 4.0},
                        .weld_system_2{.voltage                      = 31.0,
                           .current                      = 700.0,
                           .wire_lin_velocity_mm_per_sec = help_sim::CalculateWireSpeedMmPerSec(method::AC, 4., 700),
                           .deposition_rate              = 10.6,
                           .heat_input                   = 1.2,
                           .twin_wire                    = false,
                           .wire_diameter_mm             = 4.0},
                        .use_edge_sensor = true},
        .test_joint_geometry{help_sim::TEST_JOINT_GEOMETRY_DEEP},

        .testcase_parameters{.expected_beads_in_layer{
            {1, 2},
            {2, 2},
            {3, 2},
            {4, 2},
            {5, 2},
            {6, 3},
            {7, 3},
            {8, 3},
            {9, 3},
            {10, 3},
            {11, 4},
            {12, 4},
            {13, 4},
            {14, 4},
            {15, 5},
            {16, 6},
        }}
    };

    SUBCASE("Basic") { AdativeWeldingTest(test_parameters); }
    SUBCASE("WithStepUpLimits") {
      test_parameters.abp_parameters.step_up_limits = {
          32.1, /* required top groove width for 3 beads */
          42.0, /* required top groove width for 4 beads */
      };
      test_parameters.testcase_parameters.expected_beads_in_layer[6]  = 2; /* 3 -> 2 */
      test_parameters.testcase_parameters.expected_beads_in_layer[7]  = 2; /* 3 -> 2 */
      test_parameters.testcase_parameters.expected_beads_in_layer[11] = 3; /* 4 -> 3 */
      test_parameters.testcase_parameters.expected_beads_in_layer[15] = 4; /* 5 -> 4 */
      test_parameters.testcase_parameters.expected_beads_in_layer[16] = 5; /* 6 -> 5 */
      test_parameters.testcase_parameters.expected_beads_in_layer[17] = 6; /* additional layer */
      AdativeWeldingTest(test_parameters);
    }
  }

  TEST_CASE("adaptive_welding_no_adaptivity_no_cap") {
    help_sim::TestParameters test_parameters{
        .abp_parameters{.wall_offset_mm = 2.,
                        .bead_overlap   = 20.,
                        .step_up_value  = 0.5,
                        .k_gain         = 0.,
                        .heat_input{.min = 2.1, .max = 3.4},
                        .weld_system_2_current{.min = 750., .max = 750.},
                        .weld_speed{.min = 100., .max = 100.},
                        .bead_switch_angle = 15.,
                        .cap_corner_offset = 0.0,
                        .cap_beads         = 2,
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
                        .use_edge_sensor = false},
        .test_joint_geometry{help_sim::TEST_JOINT_GEOMETRY_WIDE},

        .testcase_parameters{.expected_beads_in_layer{{1, 4}, {2, 4}, {3, 5}, {4, 5}}}
    };
    AdativeWeldingTest(test_parameters);
  }
}

namespace {
auto AdativeWeldingTest(help_sim::TestParameters &test_parameters) -> void {
  const std::chrono::milliseconds time_per_step_ms{static_cast<int>(help_sim::CalculateStepTimeMs(
      test_parameters.welding_parameters.weld_object_diameter_m,
      test_parameters.welding_parameters.weld_object_speed_cm_per_min, NUMBER_OF_STEPS_PER_REV))};
  // WS1
  common::msg::weld_system::GetWeldSystemDataRsp weld_system_1_status_rsp{
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
  common::msg::weld_system::GetWeldSystemDataRsp weld_system_2_status_rsp{
      .voltage = static_cast<float>(test_parameters.welding_parameters.weld_system_2.voltage),
      .current = static_cast<float>(test_parameters.welding_parameters.weld_system_2.current),
      .wire_lin_velocity =
          static_cast<float>(test_parameters.welding_parameters.weld_system_2.wire_lin_velocity_mm_per_sec),
      .deposition_rate = static_cast<float>(test_parameters.welding_parameters.weld_system_2.deposition_rate),
      .heat_input      = static_cast<float>(test_parameters.welding_parameters.weld_system_2.heat_input),
      .twin_wire       = test_parameters.welding_parameters.weld_system_2.twin_wire,
      .wire_diameter   = static_cast<float>(test_parameters.welding_parameters.weld_system_2.wire_diameter_mm),
  };

  TestFixture fixture;

  auto weld_control_config                   = fixture.GetConfigManagerMock()->GetWeldControlConfiguration();
  weld_control_config.scanner_input_interval = std::chrono::milliseconds{1884};
  weld_control_config.adaptivity.gaussian_filter.kernel_size = 9;
  weld_control_config.adaptivity.gaussian_filter.sigma       = 3.0;
  weld_control_config.handover_grace                         = std::chrono::seconds{25};
  weld_control_config.storage_resolution =
      test_parameters.welding_parameters.weld_object_diameter_m * std::numbers::pi * 1000.0 / NUMBER_OF_STEPS_PER_REV;
  fixture.GetConfigManagerMock()->SetWeldControlConfiguration(weld_control_config);

  fixture.GetConfigManagerMock()->SetWeldControlConfiguration(weld_control_config);

  fixture.StartApplication();

  StoreSettings(fixture, TestSettings{.use_edge_sensor = test_parameters.welding_parameters.use_edge_sensor}, true);

  // Set up to use timer wrapper
  fixture.SetupTimerWrapper();

  // Adaptive variables
  auto ws1_wire_feed_speed_mm_per_s = helpers_simulator::CalculateWireSpeedMmPerSec(
      method::DC, test_parameters.welding_parameters.weld_system_1.wire_diameter_mm, weld_system_1_status_rsp.current);
  auto ws2_wire_feed_speed_mm_per_s = helpers_simulator::CalculateWireSpeedMmPerSec(
      method::AC, test_parameters.welding_parameters.weld_system_2.wire_diameter_mm, weld_system_2_status_rsp.current);

  // Create simulator
  std::unique_ptr<depsim::ISimulator> simulator = depsim::CreateSimulator();
  depsim::SimConfig sim_config                  = simulator->CreateSimConfig();
  help_sim::SetSimulatorDefault(sim_config, NUMBER_OF_STEPS_PER_REV);

  // Set up joint geometry
  help_sim::SetJointGeometry(fixture, sim_config, test_parameters.test_joint_geometry);

  sim_config.travel_speed =
      help_sim::ConvertCMPerMin2MPerS(test_parameters.welding_parameters.weld_object_speed_cm_per_min);

  // Add adaptive torches
  auto depsim_ws1_torch = simulator->AddSingleWireTorch(
      help_sim::ConvertMm2M(test_parameters.welding_parameters.weld_system_1.wire_diameter_mm),
      help_sim::ConvertMmPerS2MPerS(ws1_wire_feed_speed_mm_per_s));
  auto depsim_ws2_torch = simulator->AddSingleWireTorch(
      help_sim::ConvertMm2M(test_parameters.welding_parameters.weld_system_2.wire_diameter_mm),
      help_sim::ConvertMmPerS2MPerS(ws2_wire_feed_speed_mm_per_s));

  // Simulator config OPCS/LPCS
  help_sim::ConfigOPCS(sim_config, test_parameters.welding_parameters.weld_object_diameter_m,
                       test_parameters.welding_parameters.stickout_m);
  help_sim::ConfigLPCS(sim_config, test_parameters.welding_parameters.stickout_m, SCANNER_MOUNT_ANGLE);

  simulator->Initialize(sim_config);

  CalibrateConfig conf{.stickout_m              = test_parameters.welding_parameters.stickout_m,
                       .touch_point_depth_m     = TOUCH_POINT_DEPTH_M,
                       .scanner_mount_angle_rad = SCANNER_MOUNT_ANGLE,
                       .wire_diameter_mm        = test_parameters.welding_parameters.weld_system_1.wire_diameter_mm,
                       .weld_object_diameter_m  = test_parameters.welding_parameters.weld_object_diameter_m};

  CHECK(Calibrate(fixture, sim_config, *simulator, conf));

  simulator->Initialize(sim_config);

  depsim::Point3d torch_pos_macs(-30e-3, 0, -19e-3 + test_parameters.welding_parameters.stickout_m, depsim::MACS);
  simulator->UpdateTorchPosition(torch_pos_macs);
  TESTLOG("Set DepSim torch start position MACS, vertical {}m, horizontal: {}m", torch_pos_macs.GetX(),
          torch_pos_macs.GetZ());

  //
  // set up adaptio
  //
  auto abws        = helpers_simulator::ConvertFromOptionalAbwVector(simulator->GetAbwPoints(depsim::MACS));
  auto joint_depth = std::abs(abws[0].GetZ() - abws[1].GetZ());  // Approximate depth
  for (auto &p : abws) {
    joint_depth = std::max(std::abs(abws[0].GetZ() - p.GetZ()), joint_depth);
  }
  helpers_simulator::DumpAbw(abws);

  simulator->UpdateTorchPosition(torch_pos_macs);
  TESTLOG("Set DepSim torch position MACS (1st fill layer), horizontal {:.5f}m, vertical: {:.5f}m",
          torch_pos_macs.GetX(), torch_pos_macs.GetZ());

  help_sim::SetABPParameters(fixture, test_parameters);

  DispatchKinematicsStateChange(fixture, common::msg::kinematics::StateChange::State::HOMED);
  DispatchKinematicsEdgeStateChange(fixture, common::msg::kinematics::EdgeStateChange::State::AVAILABLE);
  CheckAndDispatchGetWeldAxis(fixture, 0.0, 0.0, help_sim::ConvertM2Mm(sim_config.joint_def_left.outer_diameter / 2.));

  // Start Joint tracking
  common::msg::management::TrackingStart start_joint_tracking_msg{
      .joint_tracking_mode = static_cast<uint32_t>(tracking::TrackingMode::TRACKING_LEFT_HEIGHT),
      .horizontal_offset   = 10.0,
      .vertical_offset     = static_cast<float>(help_sim::ConvertM2Mm(test_parameters.welding_parameters.stickout_m))};
  fixture.Management()->Dispatch(start_joint_tracking_msg);

  // Receive StartScanner
  REQUIRE_MESSAGE(fixture.Scanner()->Receive<common::msg::scanner::Start>(), "No Start msg received");

  CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "jt"});

  StartABP(fixture);
  CheckWeldControlStatus(fixture, WeldControlStatus{.weld_control_mode = "abp"});

  DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID1,
                                common::msg::weld_system::OnWeldSystemStateChange::State::ARCING);
  DispatchWeldSystemStateChange(fixture, weld_system::WeldSystemId::ID2,
                                common::msg::weld_system::OnWeldSystemStateChange::State::ARCING);

  //
  //  Execute Adaptivity Welding
  //
  auto test_ready{false};
  std::map<int, int> beads_per_layer;
  auto current_layer{1};
  auto adaptive_weld_speed_m_per_s = sim_config.travel_speed;
  bool test_stop                   = false;
  StateMonitor<std::string> state_monitor;
  while (!test_ready) {  // LastLayerCompleted(stop_layer, current_layer)) {
    abws        = helpers_simulator::ConvertFromOptionalAbwVector(simulator->GetAbwPoints(depsim::MACS));
    joint_depth = std::abs(abws[0].GetZ() - abws[1].GetZ());  // Approximate depth
    for (auto &p : abws) {
      joint_depth = std::max(std::abs(abws[0].GetZ() - p.GetZ()), joint_depth);
    }
    helpers_simulator::DumpAbw(abws);

    double current_angle{.0};
    for (int step = 0; step < NUMBER_OF_STEPS_PER_REV; step++)  // Steps per rev
    {
      abws = helpers_simulator::ConvertFromOptionalAbwVector(simulator->GetAbwPoints(depsim::LPCS));

      //
      // Update adaptio
      //
      // ABW points on scanner interface
      auto slice_data = helpers_simulator::GetSliceData(
          abws,
          static_cast<std::uint64_t>(fixture.GetClockNowFuncWrapper()->GetSystemClock().time_since_epoch().count()));
      fixture.Scanner()->Dispatch(slice_data);

      auto get_slides_position = fixture.Kinematics()->Receive<common::msg::kinematics::GetSlidesPosition>();
      fixture.Kinematics()->Dispatch(
          common::msg::kinematics::GetSlidesPositionRsp{.client_id  = get_slides_position->client_id,
                                                        .time_stamp = get_slides_position->time_stamp,
                                                        .horizontal = help_sim::ConvertM2Mm(torch_pos_macs.GetX()),
                                                        .vertical   = help_sim::ConvertM2Mm(torch_pos_macs.GetZ())});

      auto get_edge_position = fixture.Kinematics()->Receive<common::msg::kinematics::GetEdgePosition>();
      fixture.Kinematics()->Dispatch(
          common::msg::kinematics::GetEdgePositionRsp{.client_id = get_edge_position->client_id, .position = 0.0});

      TESTLOG("Set Adaptio torch position MACS, horizontal {:.5f} mm, vertical: {:.5f} mm",
              help_sim::ConvertM2Mm(torch_pos_macs.GetX()), help_sim::ConvertM2Mm(torch_pos_macs.GetZ()));

      //  Check GetWeldAxis request and dispatch response
      CheckAndDispatchGetWeldAxis(
          fixture, current_angle,
          help_sim::ConvertMPerS2RadPerS(adaptive_weld_speed_m_per_s, sim_config.joint_def_left.outer_diameter / 2.),
          help_sim::ConvertM2Mm(sim_config.joint_def_left.outer_diameter / 2.));

      // Check GetWeldSystemStatus requests for both weld-systems and send response
      weld_system_1_status_rsp.heat_input = static_cast<float>(help_sim::CalculateHeatInputValue(
          weld_system_1_status_rsp.voltage, weld_system_1_status_rsp.current, adaptive_weld_speed_m_per_s));
      weld_system_2_status_rsp.heat_input = static_cast<float>(help_sim::CalculateHeatInputValue(
          weld_system_2_status_rsp.voltage, weld_system_2_status_rsp.current, adaptive_weld_speed_m_per_s));
      TESTLOG("Calculated heatinput WeldSystem1: {:.4f} kj/mm, WeldSystem2: {:.4f} kj/mm",
              weld_system_1_status_rsp.heat_input, weld_system_2_staous_rsp.heat_input);
      CheckAndDispatchWeldSystemDataRsp(fixture, weld_system::WeldSystemId::ID1, weld_system_1_status_rsp);
      CheckAndDispatchWeldSystemDataRsp(fixture, weld_system::WeldSystemId::ID2, weld_system_2_status_rsp);

      // Recalculate adaptive variables, then update the simulator and the adaptio responses
      auto set_weld_axis = fixture.Kinematics()->Receive<common::msg::kinematics::SetWeldAxisData>();
      if (set_weld_axis) {
        adaptive_weld_speed_m_per_s =
            help_sim::ConvertRadPerS2MPerS(set_weld_axis->velocity, sim_config.joint_def_left.outer_diameter / 2.);
        TESTLOG("Recived SetWeldAxisData velocity: {:.5f} rad/s, {:.5f} m/s", set_weld_axis->velocity,
                adaptive_weld_speed_m_per_s);
        REQUIRE(adaptive_weld_speed_m_per_s >=
                doctest::Approx(help_sim::ConvertCMPerMin2MPerS(test_parameters.abp_parameters.weld_speed.min)));
        REQUIRE(adaptive_weld_speed_m_per_s <=
                help_sim::ConvertCMPerMin2MPerS(test_parameters.abp_parameters.weld_speed.max));
        simulator->UpdateTravelSpeed(adaptive_weld_speed_m_per_s);
      }
      auto set_welding_system_settings =
          fixture.WeldSystem()->Receive<common::msg::weld_system::SetWeldSystemSettings>();
      if (set_welding_system_settings) {
        TESTLOG("Recived SetWeldSystemSettings current:{}", set_welding_system_settings->current);
        REQUIRE(set_welding_system_settings->current >= test_parameters.abp_parameters.weld_system_2_current.min);
        REQUIRE(set_welding_system_settings->current <= test_parameters.abp_parameters.weld_system_2_current.max);
        ws1_wire_feed_speed_mm_per_s = help_sim::CalculateWireSpeedMmPerSec(
            method::DC, test_parameters.welding_parameters.weld_system_1.wire_diameter_mm,
            weld_system_1_status_rsp.current);
        ws2_wire_feed_speed_mm_per_s = help_sim::CalculateWireSpeedMmPerSec(
            method::AC, test_parameters.welding_parameters.weld_system_2.wire_diameter_mm,
            set_welding_system_settings->current);
        depsim_ws1_torch->SetWireFeedSpeed(help_sim::ConvertMmPerS2MPerS(ws1_wire_feed_speed_mm_per_s));
        depsim_ws2_torch->SetWireFeedSpeed(help_sim::ConvertMmPerS2MPerS(ws2_wire_feed_speed_mm_per_s));
        weld_system_1_status_rsp.wire_lin_velocity = static_cast<float>(ws1_wire_feed_speed_mm_per_s);
        weld_system_2_status_rsp.wire_lin_velocity = static_cast<float>(ws2_wire_feed_speed_mm_per_s);
        weld_system_2_status_rsp.current           = set_welding_system_settings->current;
      }
      // Dispatch expired timers
      fixture.GetTimerWrapper()->DispatchAllExpired();

      //
      // Check adaptio output
      //

      if (test_parameters.welding_parameters.use_edge_sensor &&
          fixture.Management()->Receive<common::msg::management::GracefulStop>()) {
        TESTLOG("Received GracefulStop - exit test!");
        test_stop = true;

      } else if (!test_parameters.welding_parameters.use_edge_sensor &&
                 fixture.Management()->Receive<common::msg::management::NotifyHandoverToManual>()) {
        TESTLOG("Received NotifyHandoverToManual - exit test!");
        test_stop = true;
      } else {
        // Receive SetSlidesPosition and update DepSim torch position
        auto set_slides_position = fixture.Kinematics()->Receive<common::msg::kinematics::SetSlidesPosition>();
        REQUIRE_MESSAGE(set_slides_position, "No SetSlidesPosition received");

        torch_pos_macs = depsim::Point3d(help_sim::ConvertMm2M(set_slides_position->horizontal), 0,
                                         help_sim::ConvertMm2M(set_slides_position->vertical), depsim::MACS);
        simulator->UpdateTorchPosition(torch_pos_macs);
        TESTLOG("Adaptio to set DepSim torch position MACS, horizontal {:.5f} m, vertical: {:.5f} m",
                torch_pos_macs.GetX(), torch_pos_macs.GetZ());
      }

      {  // Check ABP weld control status
        auto weld_control_status = GetWeldControlStatus(fixture);
        current_layer            = weld_control_status.layer_number.value();
        auto info_string = std::format("layer: {}, bead: {} ", current_layer, weld_control_status.bead_number.value());
        double max_standard_deviation{0.};
        double max_depth_max_deviation{0.};
        // Check if last layer completed
        if (test_stop) {
          //  Check welding by rotating weld object

          const double angle_delta_factor = 4.;
          for (double angle = 0.; angle < 2 * help_sim::PI; angle += angle_delta_factor * DELTA_ANGLE) {
            auto abws = helpers_simulator::ConvertFromOptionalAbwVector(simulator->GetAbwPoints(depsim::MACS));
            // Calculate standard deviation for point abw[1].y to abw[5].y relatively object top line
            std::vector<double> depths;
            auto object_top_line_y = [abws](double x) {
              auto k = (abws[6].GetZ() - abws[0].GetZ()) / (abws[6].GetX() - abws[0].GetX());
              return (abws[0].GetZ() - k * abws[0].GetX()) + k * x;
            };
            auto depth_mean = 0.;
            for (int i = 1; i < 6; i++) {
              depth_mean += std::abs(abws[i].GetZ() - object_top_line_y(abws[i].GetZ()));
            }
            depth_mean               = depth_mean / 5.;
            auto depth_max_deviation = 0.;
            for (int i = 1; i < 6; i++) {
              auto depth = std::abs(abws[i].GetZ() - object_top_line_y(abws[i].GetZ()));
              depths.push_back(depth);
              depth_max_deviation = std::max(depth_max_deviation, std::abs(depth_mean - depth));
            }
            auto standard_deviation = help_sim::CalculateStandardDeviation(depths, depth_mean);
            max_standard_deviation  = std::max(max_standard_deviation, standard_deviation);
            max_depth_max_deviation = std::max(max_depth_max_deviation, depth_max_deviation);
            TESTLOG("Standard deviation: {}", standard_deviation);
            TESTLOG("Depth deviation: {}", depth_max_deviation);

            // Rotate weld object without welding
            simulator->Rotate(angle_delta_factor * DELTA_ANGLE);
          }
          TESTLOG("Max standard deviation {:.5f}, max depth deviation {:.5f} ", max_standard_deviation,
                  max_depth_max_deviation);
          // End test
          test_ready = true;
          break;
        }
        if (weld_control_status.weld_control_mode.has_value()) {
          CHECK_EQ(weld_control_status.weld_control_mode.value(), "abp");
        }
        // Check bead control state by monitoring state changes
        if (step == START_BEAD_STATE_MONITORING_AT_STEP) {
          state_monitor.Clear();
        }
        if (step > START_BEAD_STATE_MONITORING_AT_STEP || step < CHECK_MONITORED_BEAD_STATE_AT_STEP) {
          state_monitor.Add(weld_control_status.bead_operation.value());
        }

        beads_per_layer[weld_control_status.layer_number.value()] = weld_control_status.bead_number.value();
      }

      auto const msg = fixture.Management()->Receive<common::msg::management::ReadyState>();
      if (msg) {
        switch (msg->state) {
          case common::msg::management::ReadyState::State::NOT_READY:
          case common::msg::management::ReadyState::State::NOT_READY_AUTO_CAL_MOVE:
          case common::msg::management::ReadyState::State::TRACKING_READY:
          case common::msg::management::ReadyState::State::ABP_READY:
            break;
          case common::msg::management::ReadyState::State::ABP_CAP_READY:
          case common::msg::management::ReadyState::State::ABP_AND_ABP_CAP_READY:
            StartABPCap(fixture);
            break;
        }
      }

      // Step position
      current_angle += DELTA_ANGLE;
      simulator->RunWithRotation(DELTA_ANGLE,
                                 test_parameters.test_joint_geometry.simulator_joint_geometry.bead_radians_m);
      // Step clocks
      fixture.GetClockNowFuncWrapper()->StepSystemClock(time_per_step_ms);
      fixture.GetClockNowFuncWrapper()->StepSteadyClock(time_per_step_ms);
    }
  }

  std::ostringstream error_string;
  error_string << "Unexpected number of beads layer\nactual:   ";
  for (auto [layer, beads] : beads_per_layer) {
    error_string << "{" << layer << ", " << beads << "} ";
  }

  error_string << "\nexpected: ";
  for (auto [layer, beads] : test_parameters.testcase_parameters.expected_beads_in_layer) {
    error_string << "{" << layer << ", " << beads << "} ";
  }

  REQUIRE_MESSAGE(beads_per_layer == test_parameters.testcase_parameters.expected_beads_in_layer, error_string.str());
}
}  // namespace
// NOLINTEND(*-magic-numbers, *-optional-access)
