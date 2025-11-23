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

inline void ProvideScannerAndKinematicsData(MultiFixture& mfx, deposition_simulator::ISimulator& simulator,
                                            const deposition_simulator::Point3d& point) {
  auto abws_lpcs  = helpers_simulator::ConvertFromOptionalAbwVector(simulator.GetAbwPoints(deposition_simulator::LPCS));
  auto slice_data = helpers_simulator::GetSliceData(abws_lpcs, simulator, NowTimeStamp(mfx.Main()));

  // update slide postion from simulator
  controller::AxisInput slide_x_postion{};
  slide_x_postion.set_position(static_cast<float>(helpers_simulator::ConvertM2Mm(point.GetX())));
  mfx.Ctrl().Sut()->OnSlideCrossXInputUpdate(slide_x_postion);

  controller::AxisInput slide_y_postion{};
  slide_y_postion.set_position(static_cast<float>(helpers_simulator::ConvertM2Mm(point.GetZ())));
  mfx.Ctrl().Sut()->OnSlideCrossYInputUpdate(slide_y_postion);

  mfx.PlcDataUpdate();

  mfx.Main().Scanner()->Dispatch(slice_data);
}

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
                                    double abw0_horizontal_touch_offset_m = 0.0) -> bool {
  // Calculate LTC parameters from sim_config
  double ltc_stickout = 0.025;  // m
  double ltc_torch_to_laser_plane_dist =
      helpers_simulator::ComputeLtcTorchToLaserPlaneDistance(sim_config.lpcs_config, ltc_stickout);
  ltc_stickout                  = helpers_simulator::ConvertM2Mm(ltc_stickout);
  ltc_torch_to_laser_plane_dist = helpers_simulator::ConvertM2Mm(ltc_torch_to_laser_plane_dist);

  // ---------- Test sequence starts here --------------

  // Position the torch with the wire at a suitable depth in the groove
  PositionTorchInGroove(simulator, conf.stickout_m, conf.touch_point_depth_m);

  // Set laser to torch calibration
  LaserTorchCalSet(mfx.Main(), {
                                   {"distanceLaserTorch", ltc_torch_to_laser_plane_dist},
                                   {"stickout",           ltc_stickout                 },
                                   {"scannerMountAngle",  conf.scanner_mount_angle_rad }
  });

  CHECK_EQ(LaserTorchCalSetRsp(mfx.Main()).at("result"), SUCCESS_PAYLOAD.at("result"));

  // Operator starts the calibration procedure
  WeldObjectCalStart(mfx.Main(), conf.wire_diameter_mm, helpers_simulator::ConvertM2Mm(conf.stickout_m),
                     helpers_simulator::ConvertM2Mm(conf.weld_object_diameter_m) / 2.0);

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
  auto calibration_result = WeldObjectCalResult(mfx.Main(), sim_config);
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

  CHECK_EQ(WeldObjectCalSetRsp(mfx.Main()).at("result"), SUCCESS_PAYLOAD.at("result"));

  CHECK(mfx.Ctrl().Mock()->adaptio_output.get_status_ready_for_tracking());

  return true;
}

// NOLINTEND(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)
