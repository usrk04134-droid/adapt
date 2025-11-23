#pragma once

#include <fmt/core.h>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "lpcs_point.h"

namespace lpcs {

enum class SliceConfidence {
  NO,
  LOW,
  MEDIUM,
  HIGH,
};

auto inline SliceConfidenceToString(SliceConfidence confidence) -> std::string {
  switch (confidence) {
    case SliceConfidence::NO:
      return "NO";
    case SliceConfidence::LOW:
      return "LOW";
    case SliceConfidence::MEDIUM:
      return "MEDIUM";
    case SliceConfidence::HIGH:
      return "HIGH";
  }

  return "invalid";
}

struct Slice {
  std::optional<std::vector<Point>> groove;
  std::vector<Point> profile;
  uint64_t time_stamp;
  double groove_area{0.0};
  SliceConfidence confidence{SliceConfidence::NO};

  auto Describe() const -> std::string;
};

inline auto Slice::Describe() const -> std::string {
  if (!groove) {
    return {};
  }

  std::string desc = "{";
  for (const auto& coord : groove.value()) {
    desc += fmt::format("{{{:.2f}, {:.2f}}},", coord.x, coord.y);
  }
  desc += fmt::format(", confidence: {}}}", SliceConfidenceToString(confidence));
  return desc;
}

}  // namespace lpcs
