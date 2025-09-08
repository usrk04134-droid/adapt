#pragma once

#include <functional>

namespace coordination {

class CalibrationStatus {
 public:
  virtual ~CalibrationStatus() = default;

  virtual auto WeldObjectCalibrationValid() const -> bool = 0;
  virtual void Subscribe(std::function<void()> on_update) = 0;
};

}  // namespace coordination
