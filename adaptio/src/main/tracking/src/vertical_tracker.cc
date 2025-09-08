#include "vertical_tracker.h"

#include <algorithm>
#include <optional>

#include "common/logging/application_log.h"
#include "macs/macs_point.h"

using tracking::VerticalTracker;

auto VerticalTracker::GetVerticalMove(double current_horizontal) const -> std::optional<double> {
  if (!line_.has_value()) {
    LOG_ERROR("Trying to get vertical tracking position with empty line");
    return {};
  }

  if (!offset_.has_value()) {
    LOG_ERROR("Trying to get vertical tracking position before setting offset");
    return {};
  }

  auto offset = offset_.value();
  // Find the two points between which the horizontal value lies
  auto iter = std::lower_bound(
      line_->begin(), line_->end(), current_horizontal,
      [](const macs::Point& coord, double target_horizontal) { return coord.horizontal > target_horizontal; });
  double joint_height{};

  if (iter == line_->begin()) {
    joint_height = iter->vertical;
  } else if (iter == line_->end()) {
    joint_height = (iter - 1)->vertical;
  } else {
    // Linear interpolation between the two coordinates
    const macs::Point& first  = *(iter - 1);
    const macs::Point& second = *iter;

    if (second.horizontal == first.horizontal) {
      return std::nullopt;
    }

    auto slope   = (second.vertical - first.vertical) / (second.horizontal - first.horizontal);
    joint_height = first.vertical + slope * (current_horizontal - first.horizontal);
  }

  return joint_height + offset;
}

void VerticalTracker::Reset() {
  offset_ = {};
  line_   = {};
}
