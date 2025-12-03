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

  helpers_simulator::AutoTorchPosition(mfx, simulator);

  auto torch_pos = simulator.GetTorchPosition(depsim::MACS);
  TESTLOG(">>>>> Starting Tracking, with torch position: {}", ToString(torch_pos));

  REQUIRE_MESSAGE(mfx.Main().Scanner()->Receive<common::msg::scanner::Start>(), "No Start msg received");
  mfx.Main().Scanner()->Dispatch(common::msg::scanner::StartRsp{.success = true});

  ProvideScannerAndKinematicsData(mfx, simulator, torch_pos);

  double previous_z = torch_pos.GetZ();

  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    mfx.PlcDataUpdate();

    auto commanded = simulator.GetTorchPosition(depsim::MACS);
    TESTLOG(">>>>> Tracking iteration {} torch position: {}", iteration, ToString(commanded));

    const double delta_z = std::fabs(commanded.GetZ() - previous_z);
    previous_z           = commanded.GetZ();

    if (delta_z < convergence_tolerance_m) {
      break;
    }

    ProvideScannerAndKinematicsData(mfx, simulator, commanded);
  }
}

// NOLINTEND(*-magic-numbers)
