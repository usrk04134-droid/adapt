#pragma once

#include <string>

namespace bead_control {

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
