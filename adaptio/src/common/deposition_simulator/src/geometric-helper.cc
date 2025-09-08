
#include "geometric-helper.h"

#include <cmath>
#include <numbers>

#include "sim-config.h"

namespace deposition_simulator::helpers {

auto ComputeAngleIncrement(double from_angle, double to_angle) -> double {
  double angle_incr = to_angle - from_angle;

  // Constrain to
  angle_incr = std::fmod(angle_incr, (2 * std::numbers::pi));

  // Convert to positive angles
  if (angle_incr < 0) {
    angle_incr = 2 * std::numbers::pi + angle_incr;
  }

  return angle_incr;
}

auto ComputeNegAngleIncrement(double from_angle, double to_angle) -> double {
  double angle_incr = to_angle - from_angle;
  angle_incr        = pmod(angle_incr, (2 * std::numbers::pi));
  return -((2 * std::numbers::pi) - angle_incr);
}

auto pmod(double num, double div) -> double { return std::fmod(std::fmod(num, div) + div, div); }

// Computes a decent zero point for MACS relative to OPCS. Somwhere nere the surface
// of the weld object at the given clock position.
auto ComputeOpcsConfigForRollerBed(double torch_clock_ang, double radius, double sitckout, double roller_sep_opcs_y)
    -> OpcsConfig {
  OpcsConfig opcs_config{};
  opcs_config.x = 0;
  opcs_config.y = -(radius + sitckout) * std::sin(torch_clock_ang);
  opcs_config.z = -(radius + sitckout) * std::cos(torch_clock_ang) -
                  0.5 * std::sqrt((4 * std::pow(radius, 2)) - std::pow(roller_sep_opcs_y, 2));  // NOLINT

  return opcs_config;
}

}  // namespace deposition_simulator::helpers
