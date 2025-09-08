#include "simulation.h"

#include <boost/outcome/result.hpp>
#include <cmath>
#include <cstdint>
#include <numbers>

#include "controller/controller.h"
#include "controller/controller_data.h"

using controller::simulation::Simulation;

namespace {
enum class SimAxisId : uint32_t {
  INVALID,
  SLIDE_CROSS_HORIZONTAL,
  SLIDE_CROSS_VERTICAL,
  WELD_AXIS,
  SIZE,
};

unsigned edge_update_count = 0;

// Edge detector becomes active after 100 updates.
// The edge oscillates ±10 mm in a sinusoidal cycle every 1000 updates.
auto EdgeValid() -> bool { return edge_update_count > 100; }
auto EdgePosition() -> float {
  return EdgeValid() ? 10.0 * std::sin(edge_update_count * 2 * std::numbers::pi / 1000.0) : 0.0;
}
}  // namespace

Simulation::Simulation() : is_connected_(false), shutdown_(false) {}

auto Simulation::Connect() -> boost::outcome_v2::result<bool> {
  is_connected_ = true;
  return true;
}

void Simulation::Disconnect() { is_connected_ = false; }

auto Simulation::IsConnected() -> bool { return is_connected_; }

auto Simulation::RetrieveInputs() -> boost::outcome_v2::result<bool> {
  // Simulation logic goes here
  ps1_.Update();
  ps2_.Update();
  kinematics_.Update();
  ++edge_update_count;

  SetInputs();

  return true;
}

void Simulation::WriteSlideCrossXOutput(controller::AxisOutput data) {
  kinematics_.commands.x.execute         = data.get_commands_execute();
  kinematics_.commands.x.stop            = data.get_commands_stop();
  kinematics_.commands.x.follow_position = data.get_commands_follow_position();
  kinematics_.commands.x.position        = data.get_position();
  kinematics_.commands.x.speed           = data.get_velocity();
}

void Simulation::WriteSlideCrossYOutput(controller::AxisOutput data) {
  kinematics_.commands.y.execute         = data.get_commands_execute();
  kinematics_.commands.y.stop            = data.get_commands_stop();
  kinematics_.commands.y.follow_position = data.get_commands_follow_position();
  kinematics_.commands.y.position        = data.get_position();
  kinematics_.commands.y.speed           = data.get_velocity();
}

void Simulation::WriteWeldAxisOutput(controller::AxisOutput data) {
  kinematics_.commands.a.execute         = data.get_commands_execute();
  kinematics_.commands.a.stop            = data.get_commands_stop();
  kinematics_.commands.a.follow_position = data.get_commands_follow_position();
  kinematics_.commands.a.position        = data.get_position();
  kinematics_.commands.a.speed           = data.get_velocity();
}

void Simulation::WritePowerSource1Output(controller::PowerSourceOutput data) {
  ps1_.commands.weld_start = data.get_commands_start();
}

void Simulation::WritePowerSource2Output(controller::PowerSourceOutput data) {
  ps2_.commands.weld_start = data.get_commands_start();
}

void Simulation::SetInputs() {
  // Power source 1
  PowerSourceInput power_source_1;
  power_source_1.set_status_ready_to_start(ps1_.status.ready_to_start);
  power_source_1.set_status_in_welding_sequence(ps1_.status.in_welding_sequence);
  power_source_1.set_status_arcing(ps1_.status.arcing);
  power_source_1.set_status_start_failure(ps1_.status.start_failure);
  power_source_1.set_status_error(ps1_.status.error);
  power_source_1.set_status_deviation_setpoint_actual(ps1_.status.deviation_setpoint_actual);
  power_source_1.set_voltage(ps1_.status.voltage);
  power_source_1.set_current(ps1_.status.current);
  power_source_1.set_deposition_rate(ps1_.status.deposition_rate);
  power_source_1.set_heat_input(ps1_.status.heat_input);
  HandlePowerSource1Input(power_source_1);

  // Power source 2
  PowerSourceInput power_source_2;
  power_source_2.set_status_ready_to_start(ps2_.status.ready_to_start);
  power_source_2.set_status_in_welding_sequence(ps2_.status.in_welding_sequence);
  power_source_2.set_status_arcing(ps2_.status.arcing);
  power_source_2.set_status_start_failure(ps2_.status.start_failure);
  power_source_2.set_status_error(ps2_.status.error);
  power_source_2.set_status_deviation_setpoint_actual(ps2_.status.deviation_setpoint_actual);
  power_source_2.set_voltage(ps2_.status.voltage);
  power_source_2.set_current(ps2_.status.current);
  power_source_2.set_deposition_rate(ps2_.status.deposition_rate);
  power_source_2.set_heat_input(ps2_.status.heat_input);
  HandlePowerSource2Input(power_source_2);

  // X-axis
  AxisInput slide_cross_x;
  slide_cross_x.set_axis_id(static_cast<uint32_t>(SimAxisId::SLIDE_CROSS_HORIZONTAL));
  slide_cross_x.set_status_busy(kinematics_.status.x.active);
  slide_cross_x.set_status_enabled(kinematics_.status.x.active);
  slide_cross_x.set_status_error(false);  // error
  slide_cross_x.set_status_in_position(kinematics_.status.x.in_position);
  slide_cross_x.set_status_type(false);  // type = longitudinal
  slide_cross_x.set_status_following_position(kinematics_.status.x.follows_position);
  slide_cross_x.set_position(kinematics_.status.x.position);
  slide_cross_x.set_velocity(kinematics_.status.x.velocity);
  slide_cross_x.set_status_homed(true);
  HandleSlideCrossXInput(slide_cross_x);

  // Y-axis
  // clang-format off
  AxisInput slide_cross_y;
  slide_cross_y.set_axis_id(static_cast<uint32_t>(SimAxisId::SLIDE_CROSS_VERTICAL));
  slide_cross_y.set_status_busy(kinematics_.status.y.active);
  slide_cross_y.set_status_enabled(kinematics_.status.y.active);
  slide_cross_y.set_status_error(false); // error
  slide_cross_y.set_status_in_position(kinematics_.status.y.in_position);
  slide_cross_y.set_status_type(false); // type = longitudinal
  slide_cross_y.set_status_following_position(kinematics_.status.y.follows_position);
  slide_cross_y.set_position(kinematics_.status.y.position);
  slide_cross_y.set_velocity(kinematics_.status.y.velocity);
  slide_cross_y.set_status_homed(true);
  HandleSlideCrossYInput(slide_cross_y);

  // Weld axis
  // clang-format off
  AxisInput weld_axis;
  weld_axis.set_axis_id(static_cast<uint32_t>(SimAxisId::WELD_AXIS));
  weld_axis.set_status_busy(kinematics_.status.a.active);
  weld_axis.set_status_enabled(kinematics_.status.a.active);
  weld_axis.set_status_error(false); // error
  weld_axis.set_status_in_position(kinematics_.status.a.in_position);
  weld_axis.set_status_type(true); // type = circular
  weld_axis.set_status_following_position(kinematics_.status.a.follows_position);
  weld_axis.set_position(kinematics_.status.a.position);
  weld_axis.set_velocity(kinematics_.status.a.velocity);
  weld_axis.set_status_homed(true);
  HandleWeldAxisInput(weld_axis);

  TrackInput tracking;
  tracking.set_weld_object_radius(1000.0);
  tracking.set_status_edge_tracker_value_valid(EdgeValid());
  tracking.set_edge_tracker_value(EdgePosition());
  HandleTrackingInput(tracking);
}
