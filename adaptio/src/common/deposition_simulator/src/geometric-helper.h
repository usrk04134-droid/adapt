#pragma once

#include "sim-config.h"
namespace deposition_simulator::helpers {

// Angles must be in [0,2PI]
auto ComputeAngleIncrement(double from_angle, double to_angle) -> double;

auto ComputeOpcsConfigForRollerBed(double torch_clock_ang, double radius, double sitckout, double roller_sep_opcs_y)
    -> OpcsConfig;

auto ComputeNegAngleIncrement(double from_angle, double to_angle) -> double;

auto pmod(double num, double div) -> double;  // NOLINT

}  // namespace deposition_simulator::helpers
