#pragma once

#include <string>

namespace bead_control {

/* 500mm  radius ->  3141 / 5.0 =  628 slots */
/* 5000mm radius -> 31415 / 5.0 = 6282 slots */
auto const STORAGE_RESOLUTION = 5.0; /* in mm */

enum class WeldSide {
  LEFT,
  RIGHT,
};

enum class BeadPlacementStrategy {
  CORNERS_FIRST,
  SIDE_TO_SIDE,
};

enum class State {
  IDLE,
  STEADY,
  OVERLAPPING,
  REPOSITIONING,
};

inline auto StateToString(State op) -> std::string {
  switch (op) {
    case State::IDLE:
      return "idle";
    case State::STEADY:
      return "steady";
    case State::OVERLAPPING:
      return "overlapping";
    case State::REPOSITIONING:
      return "repositioning";
    default:
      break;
  }

  return "invalid";
}
};  // namespace bead_control
