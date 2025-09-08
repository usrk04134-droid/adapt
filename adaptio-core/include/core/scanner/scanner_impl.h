#pragma once

#include <prometheus/counter.h>
#include <prometheus/gauge.h>
#include <prometheus/histogram.h>
#include <prometheus/registry.h>

#include <atomic>
#include <boost/asio/thread_pool.hpp>
#include <boost/outcome.hpp>
#include <ctime>
#include <functional>

#include "core/scanner/image_logger.h"
#include "core/scanner/image_provider.h"
#include "core/scanner/joint_buffer/joint_buffer.h"
#include "core/scanner/joint_model.h"
#include "core/scanner/scanner.h"
#include "joint_buffer/joint_buffer.h"

namespace core::scanner {

using LaserCallback = std::function<void(bool state)>;
using Timestamp     = std::chrono::time_point<std::chrono::high_resolution_clock>;

class ScannerImpl : public Scanner {
 public:
  /**
   * Constructs a new scanner that takes in images, processes them and puts the result in a joint model.
   *
   * @param image_provider An image provider
   * @param camera_model A camera model
   * @param joint_buffer The joint model where we save the data
   * @param laser_toggle A callback that sets the laser state
   */
  ScannerImpl(ImageProvider* image_provider, JointBufferPtr joint_buffer, LaserCallback laser_toggle,
              ScannerOutputCB* scanner_output, JointModelPtr joint_model, ImageLogger* image_logger,
              prometheus::Registry* registry);

  ScannerImpl(const ScannerImpl&)                        = delete;
  auto operator=(const ScannerImpl&) -> ScannerImpl&     = delete;
  ScannerImpl(ScannerImpl&&) noexcept                    = delete;
  auto operator=(ScannerImpl&&) noexcept -> ScannerImpl& = delete;

  ~ScannerImpl() override = default;

  auto Start(enum ScannerSensitivity sensitivity) -> boost::outcome_v2::result<void> override;
  auto Start(enum ScannerSensitivity sensitivity, bool store_image_data) -> boost::outcome_v2::result<void>;

  void Stop() override;

  void Update() override;
  void UpdateJointGeometry(const JointProperties& properties) override;
  void ImageGrabbed(std::unique_ptr<core::image::Image>) override;
  size_t CountOfReceivedImages() override;

  void ClearJointBuffer();
  auto GetJointBuffer() -> const JointBuffer*;

  static auto NewOffsetAndHeight(int top, int bottom) -> std::tuple<int, int>;

 private:
  void SetupMetrics(prometheus::Registry* registry);
  auto MedianOfRecentSlices() -> std::optional<core::scanner::JointSlice>;

  ImageProvider* image_provider_;
  JointModelPtr joint_model_;
  JointBufferPtr joint_buffer_;
  LaserCallback laser_toggle_;
  ScannerOutputCB* scanner_output_;
  ImageLogger* image_logger_;

  boost::asio::thread_pool m_threadpool;
  std::mutex m_buffer_mutex;  // Protects joint_buffer_
  std::mutex m_config_mutex;  // Protects all other members

  size_t num_received   = 0;
  Timestamp latest_sent = std::chrono::high_resolution_clock::now();
  std::optional<std::tuple<int, int>> dont_allow_fov_change_until_new_dimensions_received;
  size_t frames_since_gain_change_ = 0;
  bool store_image_data_;

  std::optional<JointProperties> empty_joint_properties_{};

  struct {
    std::map<JointModelErrorCode, prometheus::Counter*> image_errors;
    std::map<uint64_t, prometheus::Counter*> image;
    prometheus::Histogram* image_processing_time;
    prometheus::Gauge* image_consecutive_errors;
  } metrics_;
};

}  // namespace core::scanner

namespace std {
template <>
struct is_error_code_enum<core::scanner::ScannerErrorCode> : true_type {};
}  // namespace std
