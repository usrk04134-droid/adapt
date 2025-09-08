#pragma once

#include <filesystem>

#include "scanner/image/image.h"

namespace scanner::image {

class TiffHandler {
 public:
  virtual ~TiffHandler() = default;

  virtual auto Write(const Image* image, const std::filesystem::path& log_path, uint32_t x_offset, uint32_t y_offset)
      -> void = 0;

  virtual auto SetJointGeometry(const std::string& joint_geometry_yaml) -> void     = 0;
  virtual auto SetScannerCalibration(const std::string& scanner_calib_yaml) -> void = 0;
};
using TiffHandlerPtr = std::unique_ptr<TiffHandler>;
}  // namespace scanner::image
