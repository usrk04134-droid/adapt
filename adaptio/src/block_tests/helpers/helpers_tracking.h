#pragma once

#include <algorithm>
#include <doctest/doctest.h>
#include <utility>
#include <vector>

#include "block_tests/helpers/helpers_mfx_calibration.h"
#include "common/groove/point.h"
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
// Note: Uses ABW points in MACS coordinates (from GetSliceInTorchPlane) which should match
// the groove after LPCS->MCS conversion that the tracking system uses
inline auto CalculateExpectedCenterTrackingPosition(
    const std::vector<deposition_simulator::Point3d>& abw_points, float horizontal_offset_mm,
    float vertical_offset_mm) -> std::pair<double, double> {
  REQUIRE(abw_points.size() >= 7);  // Ensure we have ABW0-ABW6

  // For center tracking with BOTTOM reference, horizontal uses midpoint of ABW1 and ABW5 (lower left/right)
  // Then subtracts horizontal_offset
  const double center_horizontal = (abw_points[1].GetX() + abw_points[5].GetX()) / 2.0;  // ABW1 and ABW5
  const double expected_x = center_horizontal - helpers_simulator::ConvertMm2M(horizontal_offset_mm);

  // Vertical tracker interpolates from the groove line at the calculated horizontal position
  // Use the same algorithm as VerticalTracker::GetVerticalMove - std::lower_bound with descending order comparison
  // The groove points are expected to be in descending horizontal order (right to left: ABW0->ABW6)
  // Convert ABW points to common::Point format for compatibility with tracking algorithm
  std::vector<common::Point> groove_points;
  groove_points.reserve(abw_points.size());
  for (const auto& abw : abw_points) {
    groove_points.push_back({.horizontal = abw.GetX(), .vertical = abw.GetZ()});
  }

  // Use the same lower_bound logic as vertical_tracker.cc
  // This finds the first point where coord.horizontal <= target_horizontal
  // (i.e., the rightmost point that is at or to the left of the target)
  auto iter = std::lower_bound(
      groove_points.begin(), groove_points.end(), expected_x,
      [](const common::Point& coord, double target_horizontal) { return coord.horizontal > target_horizontal; });

  double joint_height{};
  if (iter == groove_points.begin()) {
    joint_height = iter->vertical;
  } else if (iter == groove_points.end()) {
    joint_height = (iter - 1)->vertical;
  } else {
    // Linear interpolation between the two coordinates (same as vertical_tracker.cc)
    const common::Point& first  = *(iter - 1);
    const common::Point& second = *iter;

    if (second.horizontal == first.horizontal) {
      joint_height = first.vertical;  // Fallback if points are at same X
    } else {
      auto slope   = (second.vertical - first.vertical) / (second.horizontal - first.horizontal);
      joint_height = first.vertical + slope * (expected_x - first.horizontal);
    }
  }

  const double expected_z = joint_height + helpers_simulator::ConvertMm2M(vertical_offset_mm);

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
