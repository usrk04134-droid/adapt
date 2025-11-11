#pragma once

#include <fmt/core.h>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "common/groove/groove.h"
#include "common/groove/point.h"

namespace macs {

struct Slice {
  std::optional<common::Groove> groove;
  std::vector<common::Point> line;
  uint64_t time_stamp;

  auto Describe() const -> std::string;
};

inline auto Slice::Describe() const -> std::string { return groove ? groove->ToString() : ""; }

}  // namespace macs
