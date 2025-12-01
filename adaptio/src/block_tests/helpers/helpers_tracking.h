#pragma once

#include <doctest/doctest.h>

#include "block_tests/helpers/helpers_mfx_calibration.h"
#include "controller/controller_data.h"
#include "helpers.h"
#include "helpers/helpers_simulator.h"
#include "simulator_interface.h"
#include "test_utils/testlog.h"
#include "tracking/tracking_manager.h"

namespace {
const float JT_HORIZONTAL_OFFSET = 0.0;
const float JT_VERTICAL_OFFSET   = 25e-3 * 1000 + 1.0;  // STICKOUT_M * 1000 + 1.0
}  // namespace

inline void JointTracking(MultiFixture& mfx, deposition_simulator::ISimulator& simulator) {
  auto torch_pos = simulator.GetTorchPosition(deposition_simulator::MACS);
  TESTLOG(">>>>> Starting Tracking, with torch position: {}", ToString(torch_pos));

  controller::TrackInput tracking_data;
  tracking_data.set_joint_tracking_mode(static_cast<uint32_t>(tracking::TrackingMode::TRACKING_CENTER_HEIGHT));
  tracking_data.set_horizontal_offset(JT_HORIZONTAL_OFFSET);
  tracking_data.set_vertical_offset(JT_VERTICAL_OFFSET);
  tracking_data.set_linear_object_distance(0);
  tracking_data.set_weld_object_radius(3500);
  tracking_data.set_edge_tracker_value(0.0);
  mfx.Ctrl().Sut()->OnTrackingInputUpdate(tracking_data);

  controller::AxisInput axis_data;
  axis_data.set_position(1.23);
  axis_data.set_velocity(2.55);
  mfx.Ctrl().Sut()->OnWeldAxisInputUpdate(axis_data);

  controller::AdaptioInput adaptio_input;
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
  deposition_simulator::Point3d torch_pos_macs(horizontal_pos_m, 0, vertical_pos_m, deposition_simulator::MACS);
  simulator.UpdateTorchPosition(torch_pos_macs);

  TESTLOG(">>>>> Tracking, moved to torch position: {}", ToString(torch_pos_macs));
}
