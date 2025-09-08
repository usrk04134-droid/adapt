#include "tracking_manager_impl.h"

#include <optional>

#include "common/logging/application_log.h"
#include "tracking/src/horizontal_tracker.h"
#include "tracking/src/smooth_motion.h"
#include "tracking/src/vertical_tracker.h"
#include "tracking/tracking_manager.h"

using tracking::TrackingManagerImpl;

const double HIGH_MOVEMENT_THRESHOLD = 4.;

TrackingManagerImpl::TrackingManagerImpl()
    : horizontal_tracker_(HorizontalTrackingMode::LEFT), smooth_motion_(HIGH_MOVEMENT_THRESHOLD) {}

void TrackingManagerImpl::Reset() {
  horizontal_tracker_.Reset();
  vertical_tracker_.Reset();

  wanted_horizontal_position_ = {};
  wanted_vertical_position_   = {};
}

void TrackingManagerImpl::SetHorizontalTrackerMode(TrackingMode mode) {
  switch (mode) {
    case TrackingMode::TRACKING_LEFT_HEIGHT:
      horizontal_tracker_.SetTrackingMode(HorizontalTrackingMode::LEFT);
      break;
    case TrackingMode::TRACKING_RIGHT_HEIGHT:
      horizontal_tracker_.SetTrackingMode(HorizontalTrackingMode::RIGHT);
      break;
    case TrackingMode::TRACKING_CENTER_HEIGHT:
      horizontal_tracker_.SetTrackingMode(HorizontalTrackingMode::MIDDLE);
      break;
    default:
      LOG_ERROR("Trying to set invalid or unsupported tracking mode.");
      return;
  }

  LOG_INFO("Switching joint tracking mode to {}", TrackingModeToString(mode));

  tracking_mode_ = mode;
}

void TrackingManagerImpl::SetHorizontalTrackerReference(TrackingReference reference) {
  switch (reference) {
    case tracking::TrackingReference::BOTTOM:
      horizontal_tracker_.SetTrackingReference(HorizontalTrackingReference::BOTTOM);
      break;
    case tracking::TrackingReference::TOP:
      horizontal_tracker_.SetTrackingReference(HorizontalTrackingReference::TOP);
      break;
  }

  LOG_INFO("Switching joint tracking reference to {}", TrackingReferenceToString(reference));

  tracking_reference_ = reference;
}

auto TrackingManagerImpl::Update(const Input& data) -> std::optional<Output> {
  if (data.mode != tracking_mode_) {
    SetHorizontalTrackerMode(data.mode);
    wanted_horizontal_position_ = {};
    wanted_vertical_position_   = {};
  }

  if (data.reference != tracking_reference_) {
    SetHorizontalTrackerReference(data.reference);
    wanted_horizontal_position_ = {};
    wanted_vertical_position_   = {};
  }

  switch (tracking_mode_) {
    case TrackingMode::TRACKING_LEFT_HEIGHT:
    case TrackingMode::TRACKING_RIGHT_HEIGHT:
    case TrackingMode::TRACKING_CENTER_HEIGHT:
      // Vertical tracker should track on Line MCS but that is currently not set properly.
      vertical_tracker_.SetLine(data.groove);
      horizontal_tracker_.SetJoint(data.groove);
      break;
    case TrackingMode::TRACKING_HEIGHT:
    default:
      break;
  }

  horizontal_tracker_.SetOffset(data.horizontal_offset);
  vertical_tracker_.SetOffset(data.vertical_offset);

  // Finally update position
  auto target_horizontal_res = horizontal_tracker_.GetHorizontalMove();
  if (!target_horizontal_res.has_value()) {
    LOG_DEBUG("No value from horizontal tracker, not updating target position");
    return std::nullopt;
  }

  auto target_horizontal = target_horizontal_res.value();

  if (wanted_horizontal_position_.has_value()) {
    wanted_horizontal_position_ =
        smooth_motion_.GetSmoothTarget(wanted_horizontal_position_.value(), target_horizontal);
  } else {
    wanted_horizontal_position_ = target_horizontal;
  }

  // If smooth motion (arcing) then vertical tracking should be done with currect horizontal position
  // Otherwise vertical tracking is set to wanted position
  auto horizontal_position =
      data.smooth_vertical_motion ? data.axis_position.horizontal : wanted_horizontal_position_.value();

  auto target_vertical_res = vertical_tracker_.GetVerticalMove(horizontal_position);

  if (!target_vertical_res.has_value()) {
    LOG_DEBUG("No value from vertical tracker, not updating target position");
    return std::nullopt;
  }

  auto target_vertical = target_vertical_res.value();

  if (wanted_vertical_position_.has_value()) {
    wanted_vertical_position_ = smooth_motion_.GetSmoothTarget(wanted_vertical_position_.value(), target_vertical);
  } else {
    wanted_vertical_position_ = target_vertical;
  }

  return Output{.horizontal_pos = wanted_horizontal_position_.value(),
                .vertical_pos   = wanted_vertical_position_.value()};
}
