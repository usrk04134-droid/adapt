#pragma once

#include <cmath>

#include <doctest/doctest.h>
#include <fmt/core.h>

#include "block_tests/helpers/helpers_mfx.h"
#include "controller/controller_data.h"
#include "helpers.h"
#include "helpers_simulator.h"
#include "simulator_interface.h"
#include "test_utils/testlog.h"
#include "tracking/tracking_manager.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access)

namespace depsim = deposition_simulator;
using controller::AdaptioInput;
using controller::AxisInput;
using controller::TrackInput;

inline void JointTracking(MultiFixture& mfx, depsim::ISimulator& simulator, float horizontal_offset = 0.0f,
                          float vertical_offset = 25e-3f * 1000.0f + 1.0f) {  // STICKOUT_M * 1000 + 1.0
  auto torch_pos = simulator.GetTorchPosition(depsim::MACS);
  TESTLOG(">>>>> Starting Tracking, with torch position: {}", ToString(torch_pos));

  TrackInput tracking_data;
  tracking_data.set_joint_tracking_mode(static_cast<uint32_t>(tracking::TrackingMode::TRACKING_CENTER_HEIGHT));
  tracking_data.set_horizontal_offset(horizontal_offset);
  tracking_data.set_vertical_offset(vertical_offset);
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

  // Check that the torch is roughly at the correct position
  auto final_torch_pos = simulator.GetTorchPosition(depsim::MACS);
  const double tolerance_m = 0.001;  // 1mm tolerance
  CHECK(std::abs(final_torch_pos.GetX() - torch_pos_macs.GetX()) < tolerance_m);
  CHECK(std::abs(final_torch_pos.GetZ() - torch_pos_macs.GetZ()) < tolerance_m);
}

// NOLINTEND(*-magic-numbers, *-optional-access)
