#pragma once

#include <cstdint>
#include <functional>
#include <string>

namespace coordination {

enum class ActivityStatusE : uint32_t {
  IDLE                    = 0,
  LASER_TORCH_CALIBRATION = 1,
  WELD_OBJECT_CALIBRATION = 2,
  CALIBRATION_AUTO_MOVE   = 3,
  TRACKING                = 4,
  LW_CALIBRATION          = 5,
};

inline auto ActivityStatusToString(ActivityStatusE status) -> std::string {
  switch (status) {
    case ActivityStatusE::IDLE:
      return "idle";
    case ActivityStatusE::LASER_TORCH_CALIBRATION:
      return "laser-torch-calibration";
    case ActivityStatusE::WELD_OBJECT_CALIBRATION:
      return "weld-object-calibration";
    case ActivityStatusE::CALIBRATION_AUTO_MOVE:
      return "calibration-auto-move";
    case ActivityStatusE::TRACKING:
      return "tracking";
    case ActivityStatusE::LW_CALIBRATION:
      return "lw-calibration";
    default:
      break;
  }

  return "invalid";
}

class ActivityStatus {
 public:
  virtual ~ActivityStatus() = default;

  virtual auto Get() const -> ActivityStatusE             = 0;
  virtual void Set(ActivityStatusE)                       = 0;
  virtual auto IsIdle() const -> bool                     = 0;
  virtual auto ToString() const -> std::string            = 0;
  virtual void Subscribe(std::function<void()> on_update) = 0;
};

}  // namespace coordination
