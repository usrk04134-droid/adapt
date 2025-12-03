#pragma once

#include <cmath>
#include <cstdint>

#include <doctest/doctest.h>

#include "block_tests/helpers/helpers_mfx.h"
#include "block_tests/helpers/helpers_simulator.h"
#include "common/messages/management.h"
#include "common/messages/scanner.h"
#include "tracking/tracking_manager.h"

// NOLINTBEGIN(*-magic-numbers)

namespace help_sim = helpers_simulator;
namespace depsim   = deposition_simulator;

inline void JointTracking(MultiFixture& mfx, depsim::ISimulator& simulator, float horizontal_offset_mm,
                          float vertical_offset_mm, double convergence_tolerance_m = 1e-3,
                          int max_iterations = 50) {
  TrackingPreconditions(mfx);

  common::msg::management::TrackingStart start_msg{
      .joint_tracking_mode = static_cast<uint32_t>(tracking::TrackingMode::TRACKING_CENTER_HEIGHT),
      .horizontal_offset   = horizontal_offset_mm,
      .vertical_offset     = vertical_offset_mm};
  mfx.Main().Management()->Dispatch(start_msg);

  TrackingStart(mfx);

  auto torch_pos = simulator.GetTorchPosition(depsim::MACS);
  TESTLOG(">>>>> Starting Tracking, with torch position: {}", ToString(torch_pos));

  REQUIRE_MESSAGE(mfx.Main().Scanner()->Receive<common::msg::scanner::Start>(), "No Start msg received");
  mfx.Main().Scanner()->Dispatch(common::msg::scanner::StartRsp{.success = true});

  ProvideScannerAndKinematicsData(mfx, simulator, torch_pos);

  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    const auto axis_x_mm = static_cast<double>(mfx.Ctrl().Mock()->axis_x_output.get_position());
    const auto axis_z_mm = static_cast<double>(mfx.Ctrl().Mock()->axis_y_output.get_position());

    auto const axis_x_m = help_sim::ConvertMm2M(axis_x_mm);
    auto const axis_z_m = help_sim::ConvertMm2M(axis_z_mm);

    TESTLOG("h_pos {} v_pos {}", axis_x_m, axis_z_m);

    depsim::Point3d commanded(axis_x_m, 0.0, axis_z_m, depsim::MACS);
    simulator.UpdateTorchPosition(commanded);
    TESTLOG(">>>>> Tracking, moved to torch position: {}", ToString(commanded));

    const double delta_z = std::fabs(commanded.GetZ() - torch_pos.GetZ());
    torch_pos            = commanded;

    if (delta_z < convergence_tolerance_m) {
      break;
    }

    ProvideScannerAndKinematicsData(mfx, simulator, torch_pos);
  }
}

// NOLINTEND(*-magic-numbers)
