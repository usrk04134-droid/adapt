#include "core/scanner/image_logger_impl.h"

#include <fmt/core.h>

#include <chrono>
#include <filesystem>
#include <format>
#include <mutex>
#include <string>
#include <utility>

#include "core/logging/application_log.h"
#include "core/scanner/image_logger.h"
#include "core/scanner/tiff_handler.h"
#include "core/time/format.h"

namespace core::scanner {

ImageLoggerImpl::ImageLoggerImpl(TiffHandlerPtr tiff_handler) : tiff_handler_(std::move(tiff_handler)) {}

auto ImageLoggerImpl::Update(const ImageLoggerConfig& config) -> void {
  std::unique_lock<std::mutex> lock(mutex_);

  mode_ = config.mode;

  base_log_path_                = config.path;
  sample_rate_                  = config.sample_rate;
  image_error_logging_interval_ = config.on_error_interval;

  switch (mode_) {
    case ImageLoggerConfig::Mode::DIRECT:
      log_path_direct_ = CreateDirectory();
      LOG_TRACE("Start DIRECT image logging storage location: {}", log_path_direct_.c_str());
      break;
    case ImageLoggerConfig::Mode::BUFFERED:
      LOG_TRACE("Start BUFFERED image logging. Buffer size: {}", config.buffer_size);
      image_buffer_.set_capacity(config.buffer_size);
      break;
    case ImageLoggerConfig::Mode::ON_ERROR:
      LOG_TRACE("Start ON_ERROR image logging with buffer size: {} storage location: {}", config.buffer_size,
                base_log_path_.c_str());
      image_buffer_.set_capacity(config.buffer_size);
      break;
    case ImageLoggerConfig::Mode::OFF:
      image_buffer_.clear();
      break;
  }
}

void ImageLoggerImpl::AddMetaData(const std::string& joint_geometry_yaml, const std::string& scanner_calibration_yaml) {
  std::unique_lock<std::mutex> lock(mutex_);
  tiff_handler_->SetJointGeometry(joint_geometry_yaml);
  tiff_handler_->SetScannerCalibration(scanner_calibration_yaml);
}

auto ImageLoggerImpl::LogImage(ImageLoggerEntry& entry) -> void {
  if (mutex_.try_lock()) {
    if (mode_ != ImageLoggerConfig::Mode::OFF && image_counter_++ % sample_rate_ == 0) {
      switch (mode_) {
        case ImageLoggerConfig::Mode::DIRECT:
          tiff_handler_->Write(entry.image, log_path_direct_, entry.x_offset, entry.y_offset);
          break;
        case ImageLoggerConfig::Mode::BUFFERED:
        case ImageLoggerConfig::Mode::ON_ERROR:
          image_buffer_.push_back({*entry.image, entry.x_offset, entry.y_offset});
          break;
        case ImageLoggerConfig::Mode::OFF:
          break;
      }
    }
    mutex_.unlock();
  }
}

auto ImageLoggerImpl::LogImageError(ImageLoggerEntry& entry, const std::string& reason) -> void {
  if (mode_ == ImageLoggerConfig::Mode::ON_ERROR) {
    std::unique_lock<std::mutex> lock(mutex_);
    auto const now = std::chrono::steady_clock::now();

    if (now > last_failed_image_log_ + image_error_logging_interval_) {
      auto const log_path = CreateDirectory(reason);
      FlushBufferToPath(log_path);
      tiff_handler_->Write(entry.image, log_path, entry.x_offset, entry.y_offset);

      last_failed_image_log_ = now;
    }
  } else {
    LogImage(entry);
  }
}

auto ImageLoggerImpl::CreateDirectory(std::string reason) const -> std::string {
  auto log_path =
      base_log_path_ / core::time::TimePointToString(std::chrono::system_clock::now(), core::time::FMT_PATH_MS);

  if (!reason.empty()) {
    log_path += std::format("_{}", reason);
  }

  std::filesystem::create_directories(log_path);

  return log_path;
}

auto ImageLoggerImpl::FlushBufferToPath(const std::string& log_path) -> void {
  for (auto& data : image_buffer_) {
    tiff_handler_->Write(&data.image, log_path, data.x_offset, data.y_offset);
  }

  image_buffer_.clear();
}

auto ImageLoggerImpl::FlushBuffer() -> void {
  std::unique_lock<std::mutex> lock(mutex_);
  FlushBufferToPath(CreateDirectory());
}
}  // namespace core::scanner
