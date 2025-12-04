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

inline void JointTracking(MultiFixture& mfx, deposition_simulator::ISimulator& simulator, float horizontal_offset,
                          float vertical_offset) {
  constexpr double kConvergenceToleranceM = 1e-3;
  constexpr int    kMaxIterations         = 50;

  auto torch_pos = simulator.GetTorchPosition(depsim::MACS);
  TESTLOG(">>>>> Starting Tracking, with torch position: {}", ToString(torch_pos));

  float current_vertical_offset = vertical_offset;

  TrackInput tracking_data;
  tracking_data.set_joint_tracking_mode(static_cast<uint32_t>(tracking::TrackingMode::TRACKING_CENTER_HEIGHT));
  tracking_data.set_horizontal_offset(horizontal_offset);
  tracking_data.set_vertical_offset(current_vertical_offset);
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

  auto abw_in_torch_plane =
      helpers_simulator::ConvertFromOptionalAbwVector(simulator.GetSliceInTorchPlane(depsim::MACS));
  auto expected_z = abw_in_torch_plane.at(3).GetZ() + STICKOUT_M;

  for (int iteration = 0; iteration < kMaxIterations; ++iteration) {
    mfx.PlcDataUpdate();

    auto horizontal_pos_m =
        helpers_simulator::ConvertMm2M(static_cast<double>(mfx.Ctrl().Mock()->axis_x_output.get_position()));
    auto vertical_pos_m =
        helpers_simulator::ConvertMm2M(static_cast<double>(mfx.Ctrl().Mock()->axis_y_output.get_position()));

    depsim::Point3d torch_pos_macs(horizontal_pos_m, 0, vertical_pos_m, depsim::MACS);
    simulator.UpdateTorchPosition(torch_pos_macs);

    TESTLOG(">>>>> Tracking iteration {} moved to torch position: {}", iteration, ToString(torch_pos_macs));

    const double position_error = expected_z - torch_pos_macs.GetZ();
    if (std::abs(position_error) < kConvergenceToleranceM) {
      break;
    }

    current_vertical_offset += helpers_simulator::ConvertM2Mm(position_error);
    tracking_data.set_vertical_offset(current_vertical_offset);
    mfx.Ctrl().Sut()->OnTrackingInputUpdate(tracking_data);

    previous_z = torch_pos_macs.GetZ();
    ProvideScannerAndKinematicsData(mfx, simulator, torch_pos_macs);
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access)
