#include "axis_output_plc_adapter.h"

#include <functional>
#include <optional>

#include "common/logging/application_log.h"
#include "controller/controller_data.h"

using controller::AxisOutputPlcAdapter;

AxisOutputPlcAdapter::AxisOutputPlcAdapter(const std::function<void(AxisOutput)>& plc_writer)
    : plc_writer_(plc_writer) {}

void AxisOutputPlcAdapter::OnAxisOutput(AxisOutput data) {
  if (data.get_axis_id() == 1) {
    horizontal_output_to_write_ = data;
  } else if (data.get_axis_id() == 2) {
    vertical_output_to_write_ = data;
  } else if (data.get_axis_id() == 3) {
    weld_axis_output_to_write_ = data;
  } else {
    LOG_ERROR("Invalid Axis ID");
  }
}

void AxisOutputPlcAdapter::Release() { release_ = true; }

void AxisOutputPlcAdapter::OnPlcCycleWrite() {
  if (release_) {
    if (horizontal_last_written_) {
      auto horizontal_release = horizontal_last_written_.value();
      horizontal_release.set_commands_execute(false);
      horizontal_release.set_commands_stop(true);
      plc_writer_(horizontal_release);
    }

    if (vertical_last_written_) {
      auto vertical_release = vertical_last_written_.value();
      vertical_release.set_commands_execute(false);
      vertical_release.set_commands_stop(true);
      plc_writer_(vertical_release);
    }

    if (weld_axis_last_written_) {
      auto weld_axis_release = weld_axis_last_written_.value();
      weld_axis_release.set_commands_execute(false);
      weld_axis_release.set_commands_stop(true);
      plc_writer_(weld_axis_release);
    }

    release_ = false;
    return;
  }

  if (horizontal_output_to_write_) {
    plc_writer_(*horizontal_output_to_write_);
    horizontal_last_written_    = *horizontal_output_to_write_;  // save for use in release()
    horizontal_output_to_write_ = {};
  }

  if (vertical_output_to_write_) {
    plc_writer_(*vertical_output_to_write_);
    vertical_last_written_    = *vertical_output_to_write_;  // save for use in release()
    vertical_output_to_write_ = {};
  }
  if (weld_axis_output_to_write_) {
    plc_writer_(*weld_axis_output_to_write_);
    weld_axis_last_written_    = *weld_axis_output_to_write_;  // save for use in release()
    weld_axis_output_to_write_ = {};
  }
}
