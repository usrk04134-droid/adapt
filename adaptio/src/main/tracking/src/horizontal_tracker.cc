#include "horizontal_tracker.h"

#include <algorithm>
#include <optional>

#include "common/logging/application_log.h"
#include "macs/macs_groove.h"

using tracking::HorizontalTracker;

auto HorizontalTracker::GetHorizontalMove() const -> std::optional<double> {
  if (!joint_.has_value()) {
    LOG_ERROR("Trying to get horizontal tracking position with empty joint slice");
    return {};
  }

  if (!offset_.has_value()) {
    LOG_ERROR("Trying to get horizontal tracking position before setting offset");
    return {};
  }

  auto offset = offset_.value();

  double target_horizontal{};
  double target_left{};
  double target_right{};
  auto const& left = reference_ == HorizontalTrackingReference::BOTTOM ? joint_.value()[macs::ABW_LOWER_LEFT]
                                                                       : joint_.value()[macs::ABW_UPPER_LEFT];

  auto const& right = reference_ == HorizontalTrackingReference::BOTTOM ? joint_.value()[macs::ABW_LOWER_RIGHT]
                                                                        : joint_.value()[macs::ABW_UPPER_RIGHT];

  auto const center = (left.horizontal + right.horizontal) / 2;

  switch (mode_) {
    case HorizontalTrackingMode::LEFT:
      target_left       = left.horizontal - offset;  // Track abw0/1
      target_horizontal = std::max(target_left, center);
      break;
    case HorizontalTrackingMode::MIDDLE:
      target_horizontal = center - offset;
      break;
    case HorizontalTrackingMode::RIGHT:
      target_right      = right.horizontal + offset;  // Track abw5/6
      target_horizontal = std::min(target_right, center);
      break;
  }

  return target_horizontal;
}

void HorizontalTracker::Reset() {
  offset_ = {};
  joint_  = {};
}
