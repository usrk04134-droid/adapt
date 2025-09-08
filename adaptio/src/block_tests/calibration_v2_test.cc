#include <doctest/doctest.h>
#include <fmt/core.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>

#include "common/messages/kinematics.h"
#include "common/messages/management.h"
#include "common/messages/scanner.h"
#include "common/messages/weld_system.h"
#include "helpers.h"
#include "helpers_calibration_v2.h"
#include "helpers_joint_geometry.h"
#include "helpers_kinematics.h"
#include "helpers_simulator.h"
#include "helpers_weld_system.h"
#include "point3d.h"
#include "sim-config.h"
#include "simulator_interface.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access)
// #define TESTLOG_DISABLED
#include "test_utils/testlog.h"
#include "tracking/tracking_manager.h"
#include "weld_system_client/weld_system_types.h"

namespace depsim   = deposition_simulator;
namespace help_sim = helpers_simulator;

namespace {

const int SIM_3D_OBJECT_SLICES_PER_REV{800};

const double WELD_OBJECT_DIAMETER_M        = 2.0;
const double STICKOUT_M                    = 25e-3;
const double WIRE_DIAMETER_MM              = 4.0;
const double WIRE_VELOCITY_MM_PER_SEC      = 23.0;
const double SCANNER_MOUNT_ANGLE           = 6.0 * help_sim::PI / 180.0;
const double TOUCH_POINT_DEPTH_M           = 10e-3;
const double TOP_TOUCH_HORIZONTAL_OFFSET_M = 10e-3;

// Tracking constants
const float JT_HORIZONTAL_OFFSET = 0.0;
const float JT_VERTICAL_OFFSET   = STICKOUT_M * 1000 + 1.0;

const auto SUCCESS_PAYLOAD = nlohmann::json{
    {"result", "ok"}
};

const auto FAILURE_PAYLOAD = nlohmann::json{
    {"result", "fail"}
};

const nlohmann::json LASER_TORCH_CONFIG = {
    {"distanceLaserTorch", 150.0},
    {"stickout",           25.0 },
    {"scannerMountAngle",  0.26 }
};

const nlohmann::json CAL_RESULT = {
    {"residualStandardError",  0.0015                                        },
    {"rotationCenter",         {{"c1", -28.8}, {"c2", -88.7}, {"c3", -970.9}}},
    {"torchToLpcsTranslation", {{"c1", 0.0}, {"c2", 355.1}, {"c3", 23.1}}    },
    {"weldObjectRotationAxis", {{"c1", 0.0}, {"c2", 0.0}, {"c3", 1.0}}       }
};

void JointTracking(TestFixture& fixture, depsim::ISimulator& simulator) {
  auto torch_pos = simulator.GetTorchPosition();
  TESTLOG(">>>>> Starting Tracking, with torch position: {}", ToString(torch_pos));
  DispatchKinematicsStateChange(fixture, common::msg::kinematics::StateChange::State::HOMED);
  DispatchKinematicsEdgeStateChange(fixture, common::msg::kinematics::EdgeStateChange::State::AVAILABLE);

  fixture.Management()->Dispatch(common::msg::management::TrackingStart{
      .joint_tracking_mode = static_cast<uint32_t>(tracking::TrackingMode::TRACKING_CENTER_HEIGHT),
      .horizontal_offset   = JT_HORIZONTAL_OFFSET,
      .vertical_offset     = JT_VERTICAL_OFFSET});

  CHECK(fixture.Scanner()->Receive<common::msg::scanner::Start>());

  ProvideScannerAndKinematicsData(fixture, simulator, torch_pos);

  CheckAndDispatchGetWeldAxis(fixture, 1.23, 2.55, 3500);
  const common::msg::weld_system::GetWeldSystemDataRsp weld_system_status_rsp{
      .voltage           = 31.0,
      .current           = 216.123456,
      .wire_lin_velocity = 10.1,
      .deposition_rate   = 4.5,
      .heat_input        = 125.0,
      .twin_wire         = false,
      .wire_diameter     = 1.2,
  };

  CheckAndDispatchWeldSystemDataRsp(fixture, weld_system::WeldSystemId::ID1, weld_system_status_rsp);
  CheckAndDispatchWeldSystemDataRsp(fixture, weld_system::WeldSystemId::ID2, weld_system_status_rsp);
  CheckAndDispatchEdgePosition(fixture, 0.0);

  auto set_position = fixture.Kinematics()->Receive<common::msg::kinematics::SetSlidesPosition>();
  CHECK(set_position);

  auto horizontal_pos_m = help_sim::ConvertMm2M(set_position.value().horizontal);
  auto vertical_pos_m   = help_sim::ConvertMm2M(set_position.value().vertical);

  // Update the torch position according to the request
  depsim::Point3d torch_pos_macs(horizontal_pos_m, 0, vertical_pos_m, depsim::MACS);
  simulator.UpdateTorchPosition(torch_pos_macs);

  TESTLOG(">>>>> Tracking, moved to torch position: {}", ToString(torch_pos_macs));
}

}  // namespace

TEST_SUITE("CalibrationV2") {
  TEST_CASE("basic_calibration_v2") {
    TestFixture fixture;
    fixture.StartApplication();
    // Set up to use timer wrapper
    fixture.SetupTimerWrapper();

    // Create simulator
    auto simulator  = depsim::CreateSimulator();
    auto sim_config = simulator->CreateSimConfig();
    help_sim::SetSimulatorDefault(sim_config, SIM_3D_OBJECT_SLICES_PER_REV);

    // Set up joint geometry
    help_sim::SetJointGeometry(fixture, sim_config, help_sim::TEST_JOINT_GEOMETRY_WIDE);

    // Add torch
    auto depsim_ws1_torch = simulator->AddSingleWireTorch(help_sim::ConvertMm2M(WIRE_DIAMETER_MM),
                                                          help_sim::ConvertMmPerS2MPerS(WIRE_VELOCITY_MM_PER_SEC));

    // Simulator config OPCS/LPCS
    help_sim::ConfigOPCS(sim_config, WELD_OBJECT_DIAMETER_M, STICKOUT_M);
    help_sim::ConfigLPCS(sim_config, STICKOUT_M, SCANNER_MOUNT_ANGLE);

    simulator->Initialize(sim_config);

    CalibrateConfig conf{.stickout_m              = STICKOUT_M,
                         .touch_point_depth_m     = TOUCH_POINT_DEPTH_M,
                         .scanner_mount_angle_rad = SCANNER_MOUNT_ANGLE,
                         .wire_diameter_mm        = WIRE_DIAMETER_MM,
                         .weld_object_diameter_m  = WELD_OBJECT_DIAMETER_M};

    CHECK(Calibrate(fixture, sim_config, *simulator, conf));

    JointTracking(fixture, *simulator);
  }

  TEST_CASE("basic_calibration_v2_touch_top") {
    TestFixture fixture;
    fixture.StartApplication();
    // Set up to use timer wrapper
    fixture.SetupTimerWrapper();

    // Create simulator
    auto simulator  = depsim::CreateSimulator();
    auto sim_config = simulator->CreateSimConfig();
    help_sim::SetSimulatorDefault(sim_config, SIM_3D_OBJECT_SLICES_PER_REV);

    // Set up joint geometry
    help_sim::SetJointGeometry(fixture, sim_config, help_sim::TEST_JOINT_GEOMETRY_WIDE);

    // Add torch
    auto depsim_ws1_torch = simulator->AddSingleWireTorch(help_sim::ConvertMm2M(WIRE_DIAMETER_MM),
                                                          help_sim::ConvertMmPerS2MPerS(WIRE_VELOCITY_MM_PER_SEC));

    // Simulator config OPCS/LPCS
    help_sim::ConfigOPCS(sim_config, WELD_OBJECT_DIAMETER_M, STICKOUT_M);
    help_sim::ConfigLPCS(sim_config, STICKOUT_M, SCANNER_MOUNT_ANGLE);

    simulator->Initialize(sim_config);

    CalibrateConfig conf{.stickout_m              = STICKOUT_M,
                         .touch_point_depth_m     = TOUCH_POINT_DEPTH_M,
                         .scanner_mount_angle_rad = SCANNER_MOUNT_ANGLE,
                         .wire_diameter_mm        = WIRE_DIAMETER_MM,
                         .weld_object_diameter_m  = WELD_OBJECT_DIAMETER_M};

    CHECK(Calibrate(fixture, sim_config, *simulator, conf, true, TOP_TOUCH_HORIZONTAL_OFFSET_M));

    JointTracking(fixture, *simulator);
  }

  TEST_CASE("basic_calibration_v2_touch_top_u_bevel") {
    TestFixture fixture;
    fixture.StartApplication();
    // Set up to use timer wrapper
    fixture.SetupTimerWrapper();

    // Create simulator
    auto simulator  = depsim::CreateSimulator();
    auto sim_config = simulator->CreateSimConfig();
    help_sim::SetSimulatorDefault(sim_config, SIM_3D_OBJECT_SLICES_PER_REV);

    // Set up joint geometry
    help_sim::SetJointGeometry(fixture, sim_config, help_sim::TEST_JOINT_GEOMETRY_U_BEVEL);

    // Add torch
    auto depsim_ws1_torch = simulator->AddSingleWireTorch(help_sim::ConvertMm2M(WIRE_DIAMETER_MM),
                                                          help_sim::ConvertMmPerS2MPerS(WIRE_VELOCITY_MM_PER_SEC));

    // Simulator config OPCS/LPCS
    help_sim::ConfigOPCS(sim_config, WELD_OBJECT_DIAMETER_M, STICKOUT_M);
    help_sim::ConfigLPCS(sim_config, STICKOUT_M, SCANNER_MOUNT_ANGLE);

    simulator->Initialize(sim_config);

    CalibrateConfig conf{.stickout_m              = STICKOUT_M,
                         .touch_point_depth_m     = TOUCH_POINT_DEPTH_M,
                         .scanner_mount_angle_rad = SCANNER_MOUNT_ANGLE,
                         .wire_diameter_mm        = WIRE_DIAMETER_MM,
                         .weld_object_diameter_m  = WELD_OBJECT_DIAMETER_M};

    CHECK(Calibrate(fixture, sim_config, *simulator, conf, true, TOP_TOUCH_HORIZONTAL_OFFSET_M));

    JointTracking(fixture, *simulator);
  }

  TEST_CASE("cal_v2_get_ltc_before_set") {
    TestFixture fixture;
    fixture.StartApplication();

    LaserTorchCalGet(fixture);
    CHECK_EQ(LaserTorchCalGetRsp(fixture), FAILURE_PAYLOAD);
  }

  TEST_CASE("cal_v2_set_get_ltc") {
    TestFixture fixture;
    fixture.StartApplication();

    LaserTorchCalSet(fixture, LASER_TORCH_CONFIG);

    LaserTorchCalGet(fixture);
    CHECK_EQ(LaserTorchCalGetRsp(fixture), Merge(LASER_TORCH_CONFIG, SUCCESS_PAYLOAD));
  }

  TEST_CASE("cal_v2_set_cal_result_before_ltc") {
    TestFixture fixture;
    fixture.StartApplication();

    WeldObjectCalSet(fixture, CAL_RESULT);
    CHECK_EQ(WeldObjectCalSetRsp(fixture), FAILURE_PAYLOAD);
  }

  TEST_CASE("cal_v2_misc_cal_result") {
    TestFixture fixture;
    fixture.StartApplication();

    LaserTorchCalSet(fixture, LASER_TORCH_CONFIG);
    CHECK_EQ(LaserTorchCalSetRsp(fixture), SUCCESS_PAYLOAD);

    WeldObjectCalSet(fixture, CAL_RESULT);
    CHECK_EQ(WeldObjectCalSetRsp(fixture), SUCCESS_PAYLOAD);

    WeldObjectCalGet(fixture);
    CHECK_EQ(WeldObjectCalGetRsp(fixture), Merge(CAL_RESULT, SUCCESS_PAYLOAD));

    // Set new laser torch configuration, this will remove calibration result
    LaserTorchCalSet(fixture, LASER_TORCH_CONFIG);
    CHECK_EQ(LaserTorchCalSetRsp(fixture), SUCCESS_PAYLOAD);

    WeldObjectCalGet(fixture);
    CHECK_EQ(WeldObjectCalGetRsp(fixture), FAILURE_PAYLOAD);
  }

  TEST_CASE("cal_v2_start_stop") {
    TestFixture fixture;
    fixture.StartApplication();

    auto const payload = nlohmann::json({
        {"upper_joint_width_mm",        help_sim::TEST_JOINT_GEOMETRY_WIDE.upper_joint_width_mm       },
        {"groove_depth_mm",             help_sim::TEST_JOINT_GEOMETRY_WIDE.groove_depth_mm            },
        {"left_joint_angle_rad",        help_sim::TEST_JOINT_GEOMETRY_WIDE.left_joint_angle_rad       },
        {"right_joint_angle_rad",       help_sim::TEST_JOINT_GEOMETRY_WIDE.right_joint_angle_rad      },
        {"left_max_surface_angle_rad",  help_sim::TEST_JOINT_GEOMETRY_WIDE.left_max_surface_angle_rad },
        {"right_max_surface_angle_rad", help_sim::TEST_JOINT_GEOMETRY_WIDE.right_max_surface_angle_rad},
    });

    StoreJointGeometryParams(fixture, payload, true);

    LaserTorchCalSet(fixture, LASER_TORCH_CONFIG);
    CHECK_EQ(LaserTorchCalSetRsp(fixture), SUCCESS_PAYLOAD);

    WeldObjectCalStart(fixture, WIRE_DIAMETER_MM, help_sim::ConvertM2Mm(STICKOUT_M),
                       helpers_simulator::ConvertM2Mm(WELD_OBJECT_DIAMETER_M) / 2.0);

    // Receive StartScanner
    CHECK(fixture.Scanner()->Receive<common::msg::scanner::Start>());
    fixture.Scanner()->Dispatch(common::msg::scanner::StartRsp{.success = true});

    CHECK(WeldObjectCalStartRsp(fixture));

    // Send start again, this should fail
    WeldObjectCalStart(fixture, WIRE_DIAMETER_MM, help_sim::ConvertM2Mm(STICKOUT_M),
                       helpers_simulator::ConvertM2Mm(WELD_OBJECT_DIAMETER_M) / 2.0);

    CHECK_FALSE(WeldObjectCalStartRsp(fixture));

    // Now stop, then it should work to start again
    WeldObjectCalStop(fixture);
    CHECK(WeldObjectCalStopRsp(fixture));

    WeldObjectCalStart(fixture, WIRE_DIAMETER_MM, help_sim::ConvertM2Mm(STICKOUT_M),
                       helpers_simulator::ConvertM2Mm(WELD_OBJECT_DIAMETER_M) / 2.0);

    // Receive StartScanner
    CHECK(fixture.Scanner()->Receive<common::msg::scanner::Start>());
    fixture.Scanner()->Dispatch(common::msg::scanner::StartRsp{.success = true});

    CHECK(WeldObjectCalStartRsp(fixture));
  }
}
// NOLINTEND(*-magic-numbers, *-optional-access)
