#pragma once

#include <fmt/format.h>

namespace calibration {

struct LaserTorchCalibration {
  double x{.0};
  double y{.0};
  double z{.0};
  double angle{.0};
  bool store_persistent{true};
};

struct WeldObjectCalibration {
  double y{.0};
  double z{.0};
  double x_adjustment{.0};
  bool store_persistent{true};
};

inline auto LaserTorchCalibrationToString(const LaserTorchCalibration& data) -> std::string {
  return fmt::format("x: {} y: {} z: {} angle: {}", data.x, data.y, data.z, data.angle);
}

inline auto WeldObjectCalibrationToString(const WeldObjectCalibration& data) -> std::string {
  return fmt::format("y: {} z: {} x_adjustment: {}", data.y, data.z, data.x_adjustment);
}

}  // namespace calibration
