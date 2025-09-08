#pragma once

namespace weld_control {

class WeldStateObserver {
 public:
  enum class State {
    IDLE,
    WELDING,
  };

  virtual ~WeldStateObserver() = default;

  virtual void OnWeldStateChanged(State state) = 0;
};
}  // namespace weld_control
