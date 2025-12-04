#pragma once

#include <cmath>

#include <doctest/doctest.h>
#include <fmt/core.h>

#include "block_tests/helpers/helpers_mfx_calibration.h"
#include "controller/controller_data.h"
#include "helpers.h"
#include "helpers_simulator.h"
#include "simulator_interface.h"
#include "test_utils/testlog.h"
#include "tracking/tracking_manager.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access)

namespace depsim = deposition_simulator;

const float JT_HORIZONTAL_OFFSET = 0.0;
const float JT_VERTICAL_OFFSET   = 25e-3 * 1000 + 1.0;  // STICKOUT_M * 1000 + 1.0

inline void JointTracking(MultiFixture& mfx, depsim::ISimulator& simulator) {
  JointTracking(mfx, simulator, JT_HORIZONTAL_OFFSET, JT_VERTICAL_OFFSET);
}

inline void JointTracking(MultiFixture& mfx, depsim::ISimulator& simulator, float horizontal_offset,
                          float vertical_offset) {
  constexpr double kConvergenceToleranceM = 1e-3;
  constexpr int    kMaxIterations         = 50;

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
  axis_data.set_position(1.23F);
  axis_data.set_velocity(2.55F);
  mfx.Ctrl().Sut()->OnWeldAxisInputUpdate(axis_data);

  AdaptioInput adaptio_input;
  adaptio_input.set_commands_start(true);
  adaptio_input.set_sequence_type(1);
  mfx.Ctrl().Sut()->OnAdaptioInputUpdate(adaptio_input);

  mfx.PlcDataUpdate();
  CHECK_EQ(mfx.Ctrl().Mock()->adaptio_output.get_active_sequence_type(), 1);

  ProvideScannerAndKinematicsData(mfx, simulator, torch_pos);

  double previous_z = torch_pos.GetZ();

  for (int iteration = 0; iteration < kMaxIterations; ++iteration) {
    mfx.PlcDataUpdate();

    const auto horizontal_pos_m =
        helpers_simulator::ConvertMm2M(static_cast<double>(mfx.Ctrl().Mock()->axis_x_output.get_position()));
    const auto vertical_pos_m =
        helpers_simulator::ConvertMm2M(static_cast<double>(mfx.Ctrl().Mock()->axis_y_output.get_position()));

    depsim::Point3d torch_pos_macs(horizontal_pos_m, 0, vertical_pos_m, depsim::MACS);
    simulator.UpdateTorchPosition(torch_pos_macs);

    TESTLOG(">>>>> Tracking iteration {} moved to torch position: {}", iteration, ToString(torch_pos_macs));

    // Check for convergence: exit early if position change is below tolerance
    const double z_change = std::abs(torch_pos_macs.GetZ() - previous_z);
    if (z_change < kConvergenceToleranceM) {
      TESTLOG(">>>>> Tracking converged after {} iterations (z_change: {:.6f} m)", iteration + 1, z_change);
      break;
    }

    previous_z = torch_pos_macs.GetZ();
    ProvideScannerAndKinematicsData(mfx, simulator, torch_pos_macs);
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access)
