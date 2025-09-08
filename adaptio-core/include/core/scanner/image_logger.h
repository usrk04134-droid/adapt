#pragma once

#include <chrono>
#include <filesystem>
#include <string>

#include "core/image/image.h"

namespace core::scanner {

struct ImageLoggerConfig {
  enum class Mode {
    OFF,
    DIRECT,
    BUFFERED,
    ON_ERROR,
  };

  Mode mode;
  std::filesystem::path path;                  /* use by all types */
  std::uint32_t sample_rate{};                 /* used by all types */
  std::uint32_t buffer_size{};                 /* used by types: BUFFERED and ON_ERROR */
  std::chrono::milliseconds on_error_interval; /* used by ON_ERROR */
};

struct ImageLoggerEntry {
  core::image::Image* image;
  uint32_t x_offset;
  uint32_t y_offset;
};

class ImageLogger {
 public:
  virtual ~ImageLogger() = default;

  virtual void Update(const ImageLoggerConfig& config)                                                          = 0;
  virtual void AddMetaData(const std::string& joint_geometry_yaml, const std::string& scanner_calibration_yaml) = 0;
  virtual void LogImage(ImageLoggerEntry& entry)                                                                = 0;
  virtual void LogImageError(ImageLoggerEntry& entry, const std::string& reason)                                = 0;
  virtual void FlushBuffer()                                                                                    = 0;
};
using ImageLoggerPtr = std::unique_ptr<ImageLogger>;
}  // namespace core::scanner
