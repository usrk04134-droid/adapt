#pragma once

#include <cmath>

#include <doctest/doctest.h>
#include <fmt/core.h>

#include "block_tests/helpers/helpers_calibration.h"
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

  auto abw_macs = helpers_simulator::ConvertFromOptionalAbwVector(simulator.GetSliceInTorchPlane(depsim::MACS));
  auto abw_lpcs = helpers_simulator::ConvertFromOptionalAbwVector(simulator.GetSliceInTorchPlane(depsim::LPCS));

  if (abw_macs.size() >= 4 && abw_lpcs.size() >= 4) {
    const double translation_c3_mm =
        helpers_simulator::ConvertM2Mm(abw_macs[3].GetZ() - abw_lpcs[3].GetZ());

    WeldObjectCalGet(mfx.Main());
    auto calibration_rsp = WeldObjectCalGetRsp(mfx.Main());
    if (calibration_rsp.contains("payload")) {
      auto calibration_payload = calibration_rsp.at("payload");
      calibration_payload["torchToLpcsTranslation"]["c3"] = translation_c3_mm;
      WeldObjectCalSet(mfx.Main(), calibration_payload);
      CHECK(WeldObjectCalSetRsp(mfx.Main()));
    }
  }

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

    auto horizontal_pos_m =
        helpers_simulator::ConvertMm2M(static_cast<double>(mfx.Ctrl().Mock()->axis_x_output.get_position()));
    auto vertical_pos_m =
        helpers_simulator::ConvertMm2M(static_cast<double>(mfx.Ctrl().Mock()->axis_y_output.get_position()));

    depsim::Point3d torch_pos_macs(horizontal_pos_m, 0, vertical_pos_m, depsim::MACS);
    simulator.UpdateTorchPosition(torch_pos_macs);

    TESTLOG(">>>>> Tracking iteration {} moved to torch position: {}", iteration, ToString(torch_pos_macs));

    const double delta_z = std::abs(torch_pos_macs.GetZ() - previous_z);
    previous_z           = torch_pos_macs.GetZ();

    if (delta_z < kConvergenceToleranceM) {
      break;
    }

    ProvideScannerAndKinematicsData(mfx, simulator, torch_pos_macs);
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access)
