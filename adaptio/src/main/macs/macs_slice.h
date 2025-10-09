#pragma once

#include <fmt/core.h>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "macs_groove.h"
#include "macs_point.h"

namespace macs {

struct Slice {
  std::optional<Groove> groove;
  uint64_t time_stamp;

  auto Describe() const -> std::string;
};

inline auto Slice::Describe() const -> std::string { return groove ? groove->ToString() : ""; }

}  // namespace macs
