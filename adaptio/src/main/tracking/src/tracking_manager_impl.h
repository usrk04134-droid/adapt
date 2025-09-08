#pragma once

#include <optional>

#include "horizontal_tracker.h"
#include "tracking/src/smooth_motion.h"
#include "tracking/tracking_manager.h"
#include "vertical_tracker.h"

namespace tracking {

class TrackingManagerImpl : public TrackingManager {
 public:
  TrackingManagerImpl();

  // TrackingManager
  auto Update(const Input& data) -> std::optional<Output> override;
  void Reset() override;

 private:
  void SetHorizontalTrackerMode(TrackingMode mode);
  void SetHorizontalTrackerReference(TrackingReference reference);

  HorizontalTracker horizontal_tracker_;
  VerticalTracker vertical_tracker_;
  SmoothMotion smooth_motion_;

  TrackingMode tracking_mode_{TrackingMode::TRACKING_LEFT_HEIGHT};
  TrackingReference tracking_reference_{TrackingReference::BOTTOM};
  std::optional<double> wanted_vertical_position_;
  std::optional<double> wanted_horizontal_position_;
};

}  // namespace tracking
