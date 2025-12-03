#pragma once

#include <doctest/doctest.h>

#include "block_tests/helpers/helpers_web_hmi.h"
#include "controller/controller_data.h"
#include "helpers.h"
#include "helpers_calibration.h"
#include "helpers_simulator.h"
#include "sim-config.h"
#include "simulator_interface.h"
#include "test_utils/testlog.h"

const uint32_t SEQUENCE_AUTO_CAL_MOVE = 3;
const double LTC_STICKOUT_MM          = 25.0;  // Laser-to-torch calibration stickout in millimeters

struct CalibrateConfig {
  double stickout_m{};
  double touch_point_depth_m{};
  double scanner_mount_angle_rad{};
  double wire_diameter_mm{};
};

inline auto GridMeasurementAttempt(MultiFixture& mfx, deposition_simulator::ISimulator& simulator) -> bool {
  mfx.PlcDataUpdate();

  auto horizontal_pos_m =
      helpers_simulator::ConvertMm2M(static_cast<double>(mfx.Ctrl().Mock()->axis_x_output.get_position()));
  auto vertical_pos_m =
      helpers_simulator::ConvertMm2M(static_cast<double>(mfx.Ctrl().Mock()->axis_y_output.get_position()));

  // Update the torch position according to the request
  deposition_simulator::Point3d torch_pos_macs(horizontal_pos_m, 0, vertical_pos_m, deposition_simulator::MACS);

  ProvideScannerAndKinematicsData(mfx, simulator, torch_pos_macs);

  // Dispatch timeout on stabilization timer
  mfx.Main().Timer()->Dispatch("stabilization_delay");

  // Provide scannerdata again, this will be recorded (or skipped if an extra movement gridpoint)
  ProvideScannerAndKinematicsData(mfx, simulator, torch_pos_macs);

  return ReceiveProgress(mfx.Main()) != 1.0;
}

[[nodiscard]] inline auto Calibrate(MultiFixture& mfx, deposition_simulator::SimConfig& sim_config,
                                    deposition_simulator::ISimulator& simulator, const CalibrateConfig& conf,
                                    double weld_object_diameter_m, double abw0_horizontal_touch_offset_m) -> bool {
  // Calculate LTC parameters from sim_config
  const double ltc_stickout_m = helpers_simulator::ConvertMm2M(LTC_STICKOUT_MM);
  double ltc_torch_to_laser_plane_dist_m =
      helpers_simulator::ComputeLtcTorchToLaserPlaneDistance(sim_config.lpcs_config, ltc_stickout_m);
  const double ltc_stickout_mm                  = LTC_STICKOUT_MM;
  const double ltc_torch_to_laser_plane_dist_mm = helpers_simulator::ConvertM2Mm(ltc_torch_to_laser_plane_dist_m);

  // ---------- Test sequence starts here --------------

  // Position the torch with the wire at a suitable depth in the groove
  PositionTorchInGroove(simulator, conf.stickout_m, conf.touch_point_depth_m);

  // Set laser to torch calibration
  LaserTorchCalSet(mfx.Main(), {
                                   {"distanceLaserTorch", ltc_torch_to_laser_plane_dist_mm},
                                   {"stickout",           ltc_stickout_mm                 },
                                   {"scannerMountAngle",  conf.scanner_mount_angle_rad    }
  });

  CHECK(LaserTorchCalSetRsp(mfx.Main()));

  // Operator starts the calibration procedure
  WeldObjectCalStart(mfx.Main(), conf.wire_diameter_mm, helpers_simulator::ConvertM2Mm(conf.stickout_m),
                     helpers_simulator::ConvertM2Mm(weld_object_diameter_m) / 2.0);

  // Receive StartScanner
  REQUIRE_MESSAGE(mfx.Main().Scanner()->Receive<common::msg::scanner::Start>(), "No Start msg received");
  mfx.Main().Scanner()->Dispatch(common::msg::scanner::StartRsp{.success = true});

  // This Rsp triggers the WebHmi to display the instruction to
  // touch the left wall
  CHECK(WeldObjectCalStartRsp(mfx.Main()));

  // position torch with wire tip at top of groove
  // Possibly add a simulator function to touch the top?
  PositionTorchAtTopLeftTouchPoint(simulator, conf.stickout_m, abw0_horizontal_touch_offset_m);

  auto torch_pos = simulator.GetTorchPosition(deposition_simulator::MACS);
  TESTLOG(">>>>> torch at top touch position: {}", ToString(torch_pos));

  // Operator presses the left position button
  WeldObjectCalTopPos(mfx.Main());
  ProvideScannerAndKinematicsData(mfx, simulator, torch_pos);

  CHECK(WeldObjectCalTopPosRsp(mfx.Main()));

  // restore torch to touch_point_depth
  PositionTorchInGroove(simulator, conf.stickout_m, conf.touch_point_depth_m);

  // Simulate that the operator moves the torch to touch the left wall
  simulator.TouchLeftWall(conf.stickout_m);
  torch_pos = simulator.GetTorchPosition(deposition_simulator::MACS);
  TESTLOG(">>>>> deposition_simulator moved torch to left touch position: {}", ToString(torch_pos));

  // Operator presses the left position button
  WeldObjectCalLeftPos(mfx.Main());
  ProvideScannerAndKinematicsData(mfx, simulator, torch_pos);

  CHECK(WeldObjectCalLeftPosRsp(mfx.Main()));

  // Simulate that the operator moves the torch to touch the right wall
  simulator.TouchRightWall(conf.stickout_m);
  torch_pos = simulator.GetTorchPosition(deposition_simulator::MACS);
  TESTLOG(">>>>> deposition_simulator moved torch to right touch position: {}", ToString(torch_pos));

  // Operator presses the right position button
  WeldObjectCalRightPos(mfx.Main());
  ProvideScannerAndKinematicsData(mfx, simulator, torch_pos);

  // Check Ready state
  CHECK_FALSE(mfx.Ctrl().Mock()->adaptio_output.get_status_ready_for_tracking());
  CHECK_EQ(SEQUENCE_AUTO_CAL_MOVE, mfx.Ctrl().Mock()->adaptio_output.get_active_sequence_type());

  CHECK(WeldObjectCalRightPosRsp(mfx.Main()));

  TESTLOG(">>>>> Automatic grid measurement sequence started");

  // In this part of the sequence, Adaptio controls the slides
  helpers_simulator::AutoTorchPosition(mfx, simulator);

  while (GridMeasurementAttempt(mfx, simulator)) {
    // Loop while new grid positions are requested
  }

  // The procedure is complete here
  auto calibration_result = WeldObjectCalResult(mfx.Main());
  CHECK(calibration_result.at("result") == SUCCESS_PAYLOAD.at("result"));
  auto calibration_result_payload = calibration_result.at("payload");
  TESTLOG(">>>>> WeldObjectCalResult: {}", calibration_result_payload.dump());
  auto torch_to_lpcs_translation_c2 =
      calibration_result_payload.at("torchToLpcsTranslation").at("c2").get<double>() / 1000.0;
  auto const tolerance = sim_config.lpcs_config.y * 0.01;
  REQUIRE(torch_to_lpcs_translation_c2 == doctest::Approx(sim_config.lpcs_config.y).epsilon(tolerance));

  // Check Ready state
  CHECK_FALSE(mfx.Ctrl().Mock()->adaptio_output.get_status_ready_for_tracking());

  // Now apply this calibration result
  WeldObjectCalSet(mfx.Main(), calibration_result_payload);

  CHECK(WeldObjectCalSetRsp(mfx.Main()));

  CHECK(mfx.Ctrl().Mock()->adaptio_output.get_status_ready_for_tracking());

  return true;
}

[[nodiscard]] inline auto LWCalibrate(MultiFixture& mfx, deposition_simulator::SimConfig& sim_config,
                                      deposition_simulator::ISimulator& simulator, const CalibrateConfig& conf,
                                      double abw0_horizontal_touch_offset_m) -> bool {
  // NOTE: This is a placeholder test. Redesign with support for LW in the simulator.

  const double ltc_stickout_m = helpers_simulator::ConvertMm2M(LTC_STICKOUT_MM);
  double ltc_torch_to_laser_plane_dist_m =
      helpers_simulator::ComputeLtcTorchToLaserPlaneDistance(sim_config.lpcs_config, ltc_stickout_m);
  const double ltc_stickout_mm                  = LTC_STICKOUT_MM;
  const double ltc_torch_to_laser_plane_dist_mm = helpers_simulator::ConvertM2Mm(ltc_torch_to_laser_plane_dist_m);

  PositionTorchInGroove(simulator, conf.stickout_m, conf.touch_point_depth_m);

  LWCalStart(mfx.Main(), ltc_torch_to_laser_plane_dist_mm, ltc_stickout_mm, conf.scanner_mount_angle_rad);

  REQUIRE_MESSAGE(mfx.Main().Scanner()->Receive<common::msg::scanner::Start>(), "No Start msg received");
  mfx.Main().Scanner()->Dispatch(common::msg::scanner::StartRsp{.success = true});

  // Provide scanner data to capture the single observation
  // LWCalStartRsp is sent after the observation is captured
  auto torch_pos = simulator.GetTorchPosition(deposition_simulator::MACS);
  ProvideScannerAndKinematicsData(mfx, simulator, torch_pos);

  // Check for LWCalStartRsp after observation is captured
  CHECK(LWCalStartRsp(mfx.Main()));

  PositionTorchAtTopLeftTouchPoint(simulator, conf.stickout_m, abw0_horizontal_touch_offset_m);

  torch_pos = simulator.GetTorchPosition(deposition_simulator::MACS);
  TESTLOG(">>>>> torch at top touch position: {}", ToString(torch_pos));

  LWCalTopPos(mfx.Main());
  ProvideScannerAndKinematicsData(mfx, simulator, torch_pos);

  CHECK(LWCalTopPosRsp(mfx.Main()));

  PositionTorchInGroove(simulator, conf.stickout_m, conf.touch_point_depth_m);

  // Simulate that the operator moves the torch to touch the left wall
  simulator.TouchLeftWall(conf.stickout_m);
  torch_pos = simulator.GetTorchPosition(deposition_simulator::MACS);
  TESTLOG(">>>>> deposition_simulator moved torch to left touch position: {}", ToString(torch_pos));

  LWCalLeftPos(mfx.Main());
  ProvideScannerAndKinematicsData(mfx, simulator, torch_pos);

  CHECK(LWCalLeftPosRsp(mfx.Main()));

  simulator.TouchRightWall(conf.stickout_m);
  torch_pos = simulator.GetTorchPosition(deposition_simulator::MACS);
  TESTLOG(">>>>> deposition_simulator moved torch to right touch position: {}", ToString(torch_pos));

  // Operator presses the right position button
  LWCalRightPos(mfx.Main());
  ProvideScannerAndKinematicsData(mfx, simulator, torch_pos);

  CHECK(LWCalRightPosRsp(mfx.Main()));

  auto calibration_result = LWCalResult(mfx.Main());
  CHECK(calibration_result.at("result") == SUCCESS_PAYLOAD.at("result"));
  auto calibration_result_payload = calibration_result.at("payload");
  TESTLOG(">>>>> LWCalResult: {}", calibration_result_payload.dump());

  // Apply the calibration result
  LWCalSet(mfx.Main(), calibration_result_payload);

  CHECK(LWCalSetRsp(mfx.Main()));

  return true;
}

// NOLINTEND(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)
