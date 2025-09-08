#include "../axis_output_plc_adapter.h"

#include <doctest/doctest.h>

#include <functional>
#include <optional>

#include "controller/controller_data.h"

using controller::AxisOutput;
using controller::AxisOutputPlcAdapter;

// NOLINTBEGIN(*-magic-numbers, *-optional-access)

TEST_SUITE("AxisOutputPlcAdapter_tests") {
  TEST_CASE("new_output") {
    // Setup testcase
    std::optional<AxisOutput> current_output;
    auto callback = [&current_output](AxisOutput axis_output) { current_output = axis_output; };
    auto sut      = AxisOutputPlcAdapter{callback};

    // A cycle timeout with no data, there should be no callback
    sut.OnPlcCycleWrite();
    CHECK_FALSE(current_output);

    // Data is passed and a cycle timeout
    AxisOutput data{};
    data.set_commands_execute(true);
    data.set_axis_id(2);
    sut.OnAxisOutput(data);
    sut.OnPlcCycleWrite();
    CHECK_EQ(current_output.value().get_axis_id(), 2);
    CHECK_EQ(current_output.value().get_commands_execute(), true);
    CHECK_EQ(current_output.value().get_commands_stop(), false);
    data.set_commands_execute(true);

    // Release and set new data
    // the release should take effect the next cycle
    // and the new data in the cycle after the next
    sut.Release();
    data = {};
    data.set_commands_execute(true);
    data.set_axis_id(1);
    sut.OnAxisOutput(data);
    sut.OnPlcCycleWrite();
    CHECK_EQ(current_output.value().get_commands_execute(), false);
    CHECK_EQ(current_output.value().get_commands_stop(), true);
    sut.OnPlcCycleWrite();
    CHECK_EQ(current_output.value().get_commands_execute(), true);
    CHECK_EQ(current_output.value().get_commands_stop(), false);

    // Reset the output, there should be no update with no new data
    current_output = {};
    sut.OnPlcCycleWrite();
    CHECK_FALSE(current_output);

    // Set nonzero positions then run a cycle.
    // Then release and check that positions are the same
    // when writing stop
    AxisOutput horizontal_data{};
    horizontal_data.set_commands_execute(true);
    horizontal_data.set_position(10.0);
    horizontal_data.set_axis_id(1);
    sut.OnAxisOutput(horizontal_data);

    AxisOutput vertical_data{};
    vertical_data.set_commands_execute(true);
    vertical_data.set_position(30.0);
    vertical_data.set_axis_id(2);
    sut.OnAxisOutput(vertical_data);

    AxisOutput weld_axis_data{};
    weld_axis_data.set_commands_execute(true);
    weld_axis_data.set_velocity(101.0);
    weld_axis_data.set_axis_id(3);
    sut.OnAxisOutput(weld_axis_data);

    sut.OnPlcCycleWrite();
    sut.Release();
    sut.OnPlcCycleWrite();

    // only checking weld axis since current_output gets overwritten
    CHECK_EQ(current_output.value().get_axis_id(), 3);
    CHECK_EQ(current_output.value().get_velocity(), 101.);
  }
}
// NOLINTEND(*-magic-numbers, *-optional-access)
