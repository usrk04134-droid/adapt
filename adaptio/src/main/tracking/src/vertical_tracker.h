#pragma once

#include <optional>

#include "macs/macs_groove.h"

namespace tracking {

class VerticalTracker {
 public:
  explicit VerticalTracker() = default;
  void SetLine(const macs::Groove& line) { line_ = line; };
  void SetOffset(double offset) { offset_ = offset; };
  auto GetVerticalMove(double current_horizontal) const -> std::optional<double>;
  void Reset();

 private:
  std::optional<double> offset_;
  std::optional<macs::Groove> line_;
};
}  // namespace tracking
