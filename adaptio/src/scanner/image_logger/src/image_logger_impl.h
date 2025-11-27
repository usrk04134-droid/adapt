#pragma once

#include <boost/circular_buffer.hpp>
#include <chrono>
#include <filesystem>

#include "scanner/image/tiff_handler.h"
#include "scanner/image_logger/image_logger.h"

namespace scanner::image_logger {

struct ImageLoggerBufferData {
  image::Image image;
  uint32_t x_offset;
  uint32_t y_offset;
};

class ImageLoggerImpl : public ImageLogger {
 public:
  explicit ImageLoggerImpl(image::TiffHandlerPtr tiff_handler);

  void Update(const ImageLoggerConfig& config) override;
  void AddMetaData(const std::string& joint_geometry_yaml, const std::string& scanner_calibration_yaml) override;
  void LogImage(ImageLoggerEntry& entry) override;
  void LogImageError(ImageLoggerEntry& entry, const std::string& reason) override;
  void FlushBuffer() override;

 private:
  auto CreateDirectory(std::string reason = "") const -> std::string;
  void FlushBufferToPath(const std::string& log_path);

  ImageLoggerConfig::Mode mode_ = ImageLoggerConfig::Mode::OFF;
  std::filesystem::path base_log_path_;
  std::filesystem::path log_path_direct_;
  uint32_t sample_rate_ = 1;
  std::chrono::milliseconds image_error_logging_interval_;
  boost::circular_buffer<ImageLoggerBufferData> image_buffer_;
  uint32_t image_counter_ = 0;
  image::TiffHandlerPtr tiff_handler_;
  std::mutex mutex_;
  std::chrono::steady_clock::time_point last_failed_image_log_;
};
}  // namespace scanner::image_logger
