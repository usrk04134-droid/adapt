#pragma once

#include <doctest/doctest.h>

#include "controller/controller_data.h"
#include "helpers.h"
#include "helpers_simulator.h"
#include "simulator_interface.h"
#include "test_utils/testlog.h"

// NOLINTBEGIN(*-magic-numbers, readability-function-cognitive-complexity)

inline auto TrackingPreconditions(MultiFixture& mfx) {
  controller::AxisInput axis_data;
  axis_data.set_status_homed(true);
  mfx.Ctrl().Sut()->OnWeldAxisInputUpdate(axis_data);

  controller::TrackInput tracking_data;
  tracking_data.set_status_edge_tracker_value_valid(true);
  tracking_data.set_weld_object_radius(1000);
  mfx.Ctrl().Sut()->OnTrackingInputUpdate(tracking_data);

  mfx.PlcDataUpdate();

  CHECK(mfx.Ctrl().Mock()->adaptio_output.get_status_ready_for_tracking());
}

inline auto TrackingStart(MultiFixture& mfx) {
  controller::AdaptioInput adaptio_input;
  adaptio_input.set_commands_start(true);
  adaptio_input.set_sequence_type(1);
  mfx.Ctrl().Sut()->OnAdaptioInputUpdate(adaptio_input);
  mfx.PlcDataUpdate();

  CHECK_EQ(mfx.Ctrl().Mock()->adaptio_output.get_active_sequence_type(), 1);
}

inline void ProvideScannerAndKinematicsData(MultiFixture& mfx, deposition_simulator::ISimulator& simulator,
                                            const deposition_simulator::Point3d& point) {
  auto abws_lpcs  = helpers_simulator::ConvertFromOptionalAbwVector(simulator.GetAbwPoints(deposition_simulator::LPCS));
  auto slice_data = helpers_simulator::GetSliceData(abws_lpcs, simulator, NowTimeStamp(mfx.Main()));

  // update slide postion from simulator
  controller::AxisInput slide_x_postion{};
  slide_x_postion.set_position(static_cast<float>(helpers_simulator::ConvertM2Mm(point.GetX())));
  mfx.Ctrl().Sut()->OnSlideCrossXInputUpdate(slide_x_postion);

  controller::AxisInput slide_y_postion{};
  slide_y_postion.set_position(static_cast<float>(helpers_simulator::ConvertM2Mm(point.GetZ())));
  mfx.Ctrl().Sut()->OnSlideCrossYInputUpdate(slide_y_postion);

  mfx.PlcDataUpdate();

  mfx.Main().Scanner()->Dispatch(slice_data);
}

// NOLINTEND(*-magic-numbers, readability-function-cognitive-complexity)
