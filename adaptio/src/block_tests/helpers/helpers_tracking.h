#pragma once

#include <doctest/doctest.h>
#include <utility>

#include "block_tests/helpers/helpers_mfx_calibration.h"
#include "controller/controller_data.h"
#include "helpers.h"
#include "helpers/helpers_simulator.h"
#include "simulator_interface.h"
#include "test_utils/testlog.h"
#include "tracking/tracking_manager.h"

// Calculate expected torch position for center tracking based on tracking algorithm behavior
// For center tracking with BOTTOM reference:
// - Horizontal: midpoint of ABW1 and ABW5 (lower left/right) minus horizontal_offset
// - Vertical: interpolated groove height at calculated horizontal position plus vertical_offset
inline auto CalculateExpectedCenterTrackingPosition(
    const std::vector<deposition_simulator::Point3d>& abw_points, float horizontal_offset_mm,
    float vertical_offset_mm) -> std::pair<double, double> {
  REQUIRE(abw_points.size() >= 7);  // Ensure we have ABW0-ABW6

  // For center tracking with BOTTOM reference, horizontal uses midpoint of ABW1 and ABW5 (lower left/right)
  // Then subtracts horizontal_offset
  const double center_horizontal = (abw_points[1].GetX() + abw_points[5].GetX()) / 2.0;  // ABW1 and ABW5
  const double expected_x = center_horizontal - helpers_simulator::ConvertMm2M(horizontal_offset_mm);

  // Vertical tracker interpolates from the groove line at the calculated horizontal position
  // Find the two ABW points that bracket the expected horizontal position and interpolate
  double interpolated_z = abw_points[3].GetZ();  // Default to ABW3's Z
  for (size_t i = 0; i < abw_points.size() - 1; ++i) {
    const double x1 = abw_points[i].GetX();
    const double x2 = abw_points[i + 1].GetX();
    if ((expected_x >= x1 && expected_x <= x2) || (expected_x >= x2 && expected_x <= x1)) {
      if (x2 != x1) {
        const double t = (expected_x - x1) / (x2 - x1);
        interpolated_z =
            abw_points[i].GetZ() + t * (abw_points[i + 1].GetZ() - abw_points[i].GetZ());
      }
      break;
    }
  }
  const double expected_z = interpolated_z + helpers_simulator::ConvertMm2M(vertical_offset_mm);

  return {expected_x, expected_z};
}

inline void JointTracking(MultiFixture& mfx, deposition_simulator::ISimulator& simulator, float horizontal_offset,
                           float vertical_offset) {
  auto torch_pos = simulator.GetTorchPosition(deposition_simulator::MACS);
  TESTLOG(">>>>> Starting Tracking, with torch position: {}", ToString(torch_pos));

  controller::TrackInput tracking_data;
  tracking_data.set_joint_tracking_mode(static_cast<uint32_t>(tracking::TrackingMode::TRACKING_CENTER_HEIGHT));
  tracking_data.set_horizontal_offset(horizontal_offset);
  tracking_data.set_vertical_offset(vertical_offset);
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
