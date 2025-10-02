#pragma once

#include <cstdint>
#include <optional>

#include "common/groove/groove.h"

namespace tracking {

enum class HorizontalTrackingErrorCode : uint32_t { NO_ERROR = 0, NO_VALID_JOINT_DATA_SET, EMPTY_JOINT_DATA };

enum class HorizontalTrackingMode : uint32_t {
  LEFT = 0,
  MIDDLE,
  RIGHT,
};

enum class HorizontalTrackingReference {
  BOTTOM,
  TOP,
};

class HorizontalTracker {
 public:
  explicit HorizontalTracker(HorizontalTrackingMode mode) : mode_(mode) {};
  void SetJoint(const common::groove::Groove& joint) { joint_ = joint; };
  void SetOffset(double offset) { offset_ = offset; };
  void SetTrackingMode(HorizontalTrackingMode mode) { mode_ = mode; };
  void SetTrackingReference(HorizontalTrackingReference reference) { reference_ = reference; };
  auto GetHorizontalMove() const -> std::optional<double>;
  void Reset();

 private:
  std::optional<double> offset_;
  HorizontalTrackingMode mode_;
  HorizontalTrackingReference reference_{HorizontalTrackingReference::BOTTOM};
  std::optional<common::groove::Groove> joint_;
};
}  // namespace tracking
