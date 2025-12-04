#pragma once

#include <doctest/doctest.h>

#include "helpers.h"
#include "helpers_mfx.h"
#include "helpers_simulator.h"
#include "simulator_interface.h"
#include "test_utils/testlog.h"
#include "controller/controller_data.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access)

inline void JointTracking(MultiFixture& mfx, deposition_simulator::ISimulator& simulator, float horizontal_offset_mm,
                          float vertical_offset_mm) {
  auto torch_pos_before = simulator.GetTorchPosition(deposition_simulator::MACS);
  TESTLOG(">>>>> Starting Tracking, with torch position: {}", ToString(torch_pos_before));

  // Set tracking offsets in controller input
  controller::TrackInput tracking_data;
  tracking_data.set_status_edge_tracker_value_valid(true);
  tracking_data.set_weld_object_radius(1000);
  tracking_data.set_jt_horizontal_offset(horizontal_offset_mm);
  tracking_data.set_jt_vertical_offset(vertical_offset_mm);
  tracking_data.set_jt_tracking_mode(2);  // Center tracking mode
  mfx.Ctrl().Sut()->OnTrackingInputUpdate(tracking_data);

  // Start tracking via controller input
  controller::AdaptioInput adaptio_input;
  adaptio_input.set_commands_start(true);
  adaptio_input.set_sequence_type(1);  // Tracking sequence
  mfx.Ctrl().Sut()->OnAdaptioInputUpdate(adaptio_input);

  mfx.PlcDataUpdate();

  // Receive StartScanner message
  REQUIRE_MESSAGE(mfx.Main().Scanner()->Receive<common::msg::scanner::Start>(), "No Start msg received");
  mfx.Main().Scanner()->Dispatch(common::msg::scanner::StartRsp{.success = true});

  // Allow system to process tracking start
  mfx.Main().Timer()->Dispatch("controller_periodic_update");

  // Provide scanner and kinematics data to allow tracking to converge
  const int MAX_ITERATIONS = 10;
  for (int i = 0; i < MAX_ITERATIONS; ++i) {
    auto torch_pos = simulator.GetTorchPosition(deposition_simulator::MACS);
    ProvideScannerAndKinematicsData(mfx, simulator, torch_pos);

    // Check if system has requested new position
    auto set_position = mfx.Ctrl().Mock()->axis_x_output.get_position();
    auto vertical_position = mfx.Ctrl().Mock()->axis_y_output.get_position();

    // Update simulator position based on controller output
    double horizontal_m = helpers_simulator::ConvertMm2M(static_cast<double>(set_position));
    double vertical_m = helpers_simulator::ConvertMm2M(static_cast<double>(vertical_position));
    
    deposition_simulator::Point3d new_torch_pos(horizontal_m, 0, vertical_m, deposition_simulator::MACS);
    simulator.UpdateTorchPosition(new_torch_pos);

    TESTLOG(">>>>> Tracking, moved to torch position: {}", ToString(new_torch_pos));

    mfx.Main().Timer()->Dispatch("controller_periodic_update");
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access)
