#pragma once

#include <filesystem>

#include "core/image/image.h"

namespace core::scanner {

class TiffHandler {
 public:
  virtual ~TiffHandler() = default;

  virtual auto Write(const core::image::Image* image, const std::filesystem::path& log_path, uint32_t x_offset,
                     uint32_t y_offset) -> void = 0;

  virtual auto SetJointGeometry(const std::string& joint_geometry_yaml) -> void     = 0;
  virtual auto SetScannerCalibration(const std::string& scanner_calib_yaml) -> void = 0;
};
using TiffHandlerPtr = std::unique_ptr<TiffHandler>;
}  // namespace core::scanner
