#pragma once

#include <optional>

#include "common/groove/groove.h"

namespace tracking {

class VerticalTracker {
 public:
  explicit VerticalTracker() = default;
  void SetLine(const common::Groove& line) { line_ = line; };
  void SetOffset(double offset) { offset_ = offset; };
  auto GetVerticalMove(double current_horizontal) const -> std::optional<double>;
  void Reset();

 private:
  std::optional<double> offset_;
  std::optional<common::Groove> line_;
};
}  // namespace tracking
