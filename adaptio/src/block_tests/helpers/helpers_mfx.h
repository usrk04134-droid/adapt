#pragma once

#include <doctest/doctest.h>

#include "controller/controller_data.h"
#include "helpers.h"

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

// NOLINTEND(*-magic-numbers, readability-function-cognitive-complexity)
