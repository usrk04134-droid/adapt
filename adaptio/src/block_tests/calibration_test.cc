#include <doctest/doctest.h>
#include <fmt/core.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <nlohmann/json_fwd.hpp>

#include "block_tests/helpers/helpers_calibration.h"
#include "block_tests/helpers/helpers_web_hmi.h"
#include "common/messages/scanner.h"
#include "controller/controller_data.h"
#include "helpers/helpers.h"
#include "helpers/helpers_joint_geometry.h"
#include "helpers/helpers_mfx_calibration.h"
#include "helpers/helpers_settings.h"
#include "helpers/helpers_simulator.h"
#include "point3d.h"
#include "simulator_interface.h"
#include "tracking/tracking_manager.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access)
// #define TESTLOG_DISABLED
#include "test_utils/testlog.h"

namespace depsim   = deposition_simulator;
namespace help_sim = helpers_simulator;

using controller::AdaptioInput;
using controller::AxisInput;
using controller::TrackInput;

namespace {

const int SIM_3D_OBJECT_SLICES_PER_REV{800};

const double WELD_OBJECT_DIAMETER_M        = 2.0;
const double STICKOUT_M                    = 25e-3;
const double WIRE_DIAMETER_MM              = 4.0;
const double WIRE_VELOCITY_MM_PER_SEC      = 23.0;
const double SCANNER_MOUNT_ANGLE           = 6.0 * help_sim::PI / 180.0;
const double TOUCH_POINT_DEPTH_M           = 10e-3;
const double TOP_TOUCH_HORIZONTAL_OFFSET_M = 10e-3;

const float JT_HORIZONTAL_OFFSET = 0.0;
const float JT_VERTICAL_OFFSET   = STICKOUT_M * 1000 + 1.0;

const nlohmann::json DEFAULT_LASER_TORCH_CONFIG = {
    {"distanceLaserTorch", 350.0},
    {"stickout",           25.0 },
    {"scannerMountAngle",  0.10 }
};

const nlohmann::json DEFAULT_CAL_RESULT = {
    {"residualStandardError",  0.00                                               },
    {"rotationCenter",         {{"c1", -28.79}, {"c2", -115.95}, {"c3", -1028.50}}},
    {"torchToLpcsTranslation", {{"c1", 0.0}, {"c2", 349.29}, {"c3", -31.79}}      },
    {"weldObjectRadius",       1000.0                                             },
    {"weldObjectRotationAxis", {{"c1", 1.0}, {"c2", 0.0}, {"c3", 0.0}}            }
};

void JointTracking(MultiFixture& mfx, depsim::ISimulator& simulator) {
  auto torch_pos = simulator.GetTorchPosition(depsim::MACS);
  TESTLOG(">>>>> Starting Tracking, with torch position: {}", ToString(torch_pos));

  TrackInput tracking_data;
  tracking_data.set_joint_tracking_mode(static_cast<uint32_t>(tracking::TrackingMode::TRACKING_CENTER_HEIGHT));
  tracking_data.set_horizontal_offset(JT_HORIZONTAL_OFFSET);
  tracking_data.set_vertical_offset(JT_VERTICAL_OFFSET);
  tracking_data.set_linear_object_distance(0);
  tracking_data.set_weld_object_radius(3500);
  tracking_data.set_edge_tracker_value(0.0);
  mfx.Ctrl().Sut()->OnTrackingInputUpdate(tracking_data);

  AxisInput axis_data;
  axis_data.set_position(1.23);
  axis_data.set_velocity(2.55);
  mfx.Ctrl().Sut()->OnWeldAxisInputUpdate(axis_data);

  AdaptioInput adaptio_input;
  adaptio_input.set_commands_start(true);
  adaptio_input.set_sequence_type(1);
  mfx.Ctrl().Sut()->OnAdaptioInputUpdate(adaptio_input);

  mfx.PlcDataUpdate();
  CHECK_EQ(mfx.Ctrl().Mock()->adaptio_output.get_active_sequence_type(), 1);

  ProvideScannerAndKinematicsData(mfx, simulator, torch_pos);

  auto horizontal_pos_m =
      helpers_simulator::ConvertMm2M(static_cast<double>(mfx.Ctrl().Mock()->axis_x_output.get_position()));
  auto vertical_pos_m =
      helpers_simulator::ConvertMm2M(static_cast<double>(mfx.Ctrl().Mock()->axis_y_output.get_position()));

  // Update the torch position according to the request
  depsim::Point3d torch_pos_macs(horizontal_pos_m, 0, vertical_pos_m, depsim::MACS);
  simulator.UpdateTorchPosition(torch_pos_macs);

  TESTLOG(">>>>> Tracking, moved to torch position: {}", ToString(torch_pos_macs));
}

}  // namespace

TEST_SUITE("MultiblockCalibration") {
  TEST_CASE("basic_calibration") {
    MultiFixture mfx;

    auto simulator  = depsim::CreateSimulator();
    auto sim_config = simulator->CreateSimConfig();
    help_sim::SetSimulatorDefault(sim_config, SIM_3D_OBJECT_SLICES_PER_REV);

    help_sim::SetJointGeometry(mfx.Main(), sim_config, help_sim::TEST_JOINT_GEOMETRY_WIDE);
    StoreSettings(mfx.Main(), TestSettings{.use_edge_sensor = false}, true);

    auto depsim_ws1_torch = simulator->AddSingleWireTorch(help_sim::ConvertMm2M(WIRE_DIAMETER_MM),
                                                          help_sim::ConvertMmPerS2MPerS(WIRE_VELOCITY_MM_PER_SEC));

    help_sim::ConfigOPCS(sim_config, WELD_OBJECT_DIAMETER_M, STICKOUT_M);
    help_sim::ConfigLPCS(sim_config, STICKOUT_M, SCANNER_MOUNT_ANGLE);

    simulator->Initialize(sim_config);

    CalibrateConfig conf{.stickout_m              = STICKOUT_M,
                         .touch_point_depth_m     = TOUCH_POINT_DEPTH_M,
                         .scanner_mount_angle_rad = SCANNER_MOUNT_ANGLE,
                         .wire_diameter_mm        = WIRE_DIAMETER_MM,
                         .weld_object_diameter_m  = WELD_OBJECT_DIAMETER_M};

    CHECK(Calibrate(mfx, sim_config, *simulator, conf));

    JointTracking(mfx, *simulator);
  }

  TEST_CASE("basic_calibration_touch_top") {
    MultiFixture mfx;

    auto simulator  = depsim::CreateSimulator();
    auto sim_config = simulator->CreateSimConfig();
    help_sim::SetSimulatorDefault(sim_config, SIM_3D_OBJECT_SLICES_PER_REV);

    help_sim::SetJointGeometry(mfx.Main(), sim_config, help_sim::TEST_JOINT_GEOMETRY_WIDE);
    StoreSettings(mfx.Main(), TestSettings{.use_edge_sensor = false}, true);

    auto depsim_ws1_torch = simulator->AddSingleWireTorch(help_sim::ConvertMm2M(WIRE_DIAMETER_MM),
                                                          help_sim::ConvertMmPerS2MPerS(WIRE_VELOCITY_MM_PER_SEC));

    help_sim::ConfigOPCS(sim_config, WELD_OBJECT_DIAMETER_M, STICKOUT_M);
    help_sim::ConfigLPCS(sim_config, STICKOUT_M, SCANNER_MOUNT_ANGLE);

    simulator->Initialize(sim_config);

    CalibrateConfig conf{.stickout_m              = STICKOUT_M,
                         .touch_point_depth_m     = TOUCH_POINT_DEPTH_M,
                         .scanner_mount_angle_rad = SCANNER_MOUNT_ANGLE,
                         .wire_diameter_mm        = WIRE_DIAMETER_MM,
                         .weld_object_diameter_m  = WELD_OBJECT_DIAMETER_M};

    CHECK(Calibrate(mfx, sim_config, *simulator, conf, TOP_TOUCH_HORIZONTAL_OFFSET_M));

    JointTracking(mfx, *simulator);
  }

  TEST_CASE("basic_calibration_touch_top_u_bevel") {
    MultiFixture mfx;

    auto simulator  = depsim::CreateSimulator();
    auto sim_config = simulator->CreateSimConfig();
    help_sim::SetSimulatorDefault(sim_config, SIM_3D_OBJECT_SLICES_PER_REV);

    help_sim::SetJointGeometry(mfx.Main(), sim_config, help_sim::TEST_JOINT_GEOMETRY_U_BEVEL);
    StoreSettings(mfx.Main(), TestSettings{.use_edge_sensor = false}, true);

    auto depsim_ws1_torch = simulator->AddSingleWireTorch(help_sim::ConvertMm2M(WIRE_DIAMETER_MM),
                                                          help_sim::ConvertMmPerS2MPerS(WIRE_VELOCITY_MM_PER_SEC));

    help_sim::ConfigOPCS(sim_config, WELD_OBJECT_DIAMETER_M, STICKOUT_M);
    help_sim::ConfigLPCS(sim_config, STICKOUT_M, SCANNER_MOUNT_ANGLE);

    simulator->Initialize(sim_config);

    CalibrateConfig conf{.stickout_m              = STICKOUT_M,
                         .touch_point_depth_m     = TOUCH_POINT_DEPTH_M,
                         .scanner_mount_angle_rad = SCANNER_MOUNT_ANGLE,
                         .wire_diameter_mm        = WIRE_DIAMETER_MM,
                         .weld_object_diameter_m  = WELD_OBJECT_DIAMETER_M};

    CHECK(Calibrate(mfx, sim_config, *simulator, conf, TOP_TOUCH_HORIZONTAL_OFFSET_M));

    JointTracking(mfx, *simulator);
  }

  TEST_CASE("cal_set_get_ltc") {
    MultiFixture mfx;

    LaserTorchCalSet(mfx.Main(), DEFAULT_LASER_TORCH_CONFIG);

    LaserTorchCalGet(mfx.Main());
    CHECK_EQ(LaserTorchCalGetRsp(mfx.Main()), Merge(
                                                  nlohmann::json{
                                                      {"payload", DEFAULT_LASER_TORCH_CONFIG}
    },
                                                  SUCCESS_PAYLOAD));
  }

  TEST_CASE("cal_misc_cal_result") {
    MultiFixture mfx;

    // Set new laser torch configuration, this will remove calibration result
    LaserTorchCalSet(mfx.Main(), DEFAULT_LASER_TORCH_CONFIG);
    CHECK_EQ(LaserTorchCalSetRsp(mfx.Main()).at("result"), SUCCESS_PAYLOAD.at("result"));

    WeldObjectCalGet(mfx.Main());
    CHECK_EQ(WeldObjectCalGetRsp(mfx.Main()).at("result"), FAILURE_PAYLOAD.at("result"));
  }

  TEST_CASE("cal_start_stop") {
    MultiFixture mfx;

    auto const payload = nlohmann::json({
        {"upperJointWidthMm",       help_sim::TEST_JOINT_GEOMETRY_WIDE.upper_joint_width_mm       },
        {"grooveDepthMm",           help_sim::TEST_JOINT_GEOMETRY_WIDE.groove_depth_mm            },
        {"leftJointAngleRad",       help_sim::TEST_JOINT_GEOMETRY_WIDE.left_joint_angle_rad       },
        {"rightJointAngleRad",      help_sim::TEST_JOINT_GEOMETRY_WIDE.right_joint_angle_rad      },
        {"leftMaxSurfaceAngleRad",  help_sim::TEST_JOINT_GEOMETRY_WIDE.left_max_surface_angle_rad },
        {"rightMaxSurfaceAngleRad", help_sim::TEST_JOINT_GEOMETRY_WIDE.right_max_surface_angle_rad},
    });

    StoreJointGeometryParams(mfx.Main(), payload, true);

    LaserTorchCalSet(mfx.Main(), DEFAULT_LASER_TORCH_CONFIG);
    CHECK_EQ(LaserTorchCalSetRsp(mfx.Main()).at("result"), SUCCESS_PAYLOAD.at("result"));

    WeldObjectCalStart(mfx.Main(), WIRE_DIAMETER_MM, help_sim::ConvertM2Mm(STICKOUT_M),
                       helpers_simulator::ConvertM2Mm(WELD_OBJECT_DIAMETER_M) / 2.0);

    // Receive StartScanner
    REQUIRE_MESSAGE(mfx.Main().Scanner()->Receive<common::msg::scanner::Start>(), "No Start msg received");
    mfx.Main().Scanner()->Dispatch(common::msg::scanner::StartRsp{.success = true});

    CHECK(WeldObjectCalStartRsp(mfx.Main()));

    // Send start again, this should fail
    WeldObjectCalStart(mfx.Main(), WIRE_DIAMETER_MM, help_sim::ConvertM2Mm(STICKOUT_M),
                       helpers_simulator::ConvertM2Mm(WELD_OBJECT_DIAMETER_M) / 2.0);

    CHECK_FALSE(WeldObjectCalStartRsp(mfx.Main()));

    // Now stop, then it should work to start again
    WeldObjectCalStop(mfx.Main());
    CHECK(WeldObjectCalStopRsp(mfx.Main()));

    WeldObjectCalStart(mfx.Main(), WIRE_DIAMETER_MM, help_sim::ConvertM2Mm(STICKOUT_M),
                       helpers_simulator::ConvertM2Mm(WELD_OBJECT_DIAMETER_M) / 2.0);

    // Receive StartScanner
    REQUIRE_MESSAGE(mfx.Main().Scanner()->Receive<common::msg::scanner::Start>(), "No Start msg received");
    mfx.Main().Scanner()->Dispatch(common::msg::scanner::StartRsp{.success = true});

    CHECK(WeldObjectCalStartRsp(mfx.Main()));
  }
}
// NOLINTEND(*-magic-numbers, *-optional-access)
