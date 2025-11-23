#pragma once

#include <prometheus/counter.h>
#include <prometheus/gauge.h>
#include <prometheus/histogram.h>
#include <prometheus/registry.h>

#include <boost/asio/thread_pool.hpp>
#include <boost/outcome.hpp>
#include <ctime>
#include <functional>

#include "common/groove/groove.h"
#include "scanner/core/src/scanner.h"
#include "scanner/image_logger/image_logger.h"
#include "scanner/image_provider/image_provider.h"
#include "scanner/joint_model/joint_model.h"
#include "scanner/slice_provider/slice_provider.h"

namespace scanner {

auto const WINDOW_MARGIN      = 100;
auto const MOVE_MARGIN        = 40;
auto const MINIMUM_FOV_HEIGHT = 500;

using LaserCallback = std::function<void(bool state)>;
using Timestamp     = std::chrono::time_point<std::chrono::high_resolution_clock>;

class ScannerImpl : public Scanner {
 public:
  /**
   * Constructs a new scanner that takes in images, processes them and puts the result in a joint model.
   *
   * @param image_provider An image provider
   * @param camera_model   A camera model
   * @param slice_provider The slice provider
   * @param laser_toggle   A callback that sets the laser state
   */
  ScannerImpl(image_provider::ImageProvider* image_provider, slice_provider::SliceProviderPtr slice_provider,
              LaserCallback laser_toggle, ScannerOutputCB* scanner_output, joint_model::JointModelPtr joint_model,
              image_logger::ImageLogger* image_logger, prometheus::Registry* registry);

  ScannerImpl(const ScannerImpl&)                        = delete;
  auto operator=(const ScannerImpl&) -> ScannerImpl&     = delete;
  ScannerImpl(ScannerImpl&&) noexcept                    = delete;
  auto operator=(ScannerImpl&&) noexcept -> ScannerImpl& = delete;

  ~ScannerImpl() override = default;

  auto Start(enum ScannerSensitivity sensitivity) -> boost::outcome_v2::result<void> override;
  auto Start(enum ScannerSensitivity sensitivity, bool store_image_data) -> boost::outcome_v2::result<void>;

  void Stop() override;

  void Update() override;
  void UpdateJointApproximation(const joint_model::JointProperties& properties,
                                const std::tuple<double, double>& abw0_abw6_horizontal) override;
  void ImageGrabbed(std::unique_ptr<image::Image> image) override;
  size_t CountOfReceivedImages() override;

  static auto NewOffsetAndHeight(int top, int bottom) -> std::tuple<int, int>;

  // Test-only hook: let tests run image processing in same thread
  void SetPostExecutorForTests(std::function<void(std::function<void()>)> exec);

 private:
  void SetupMetrics(prometheus::Registry* registry);

  image_provider::ImageProvider* image_provider_;
  joint_model::JointModelPtr joint_model_;
  slice_provider::SliceProviderPtr slice_provider_;
  LaserCallback laser_toggle_;
  ScannerOutputCB* scanner_output_;
  image_logger::ImageLogger* image_logger_;
  std::optional<std::tuple<double, double>> maybe_abw0_abw6_horizontal_;

  boost::asio::thread_pool m_threadpool;
  std::mutex m_buffer_mutex;  // Protects joint_buffer_
  std::mutex m_config_mutex;  // Protects all other members

  size_t num_received   = 0;
  Timestamp latest_sent = std::chrono::high_resolution_clock::now();
  std::optional<std::tuple<int, int>> dont_allow_fov_change_until_new_dimensions_received;
  size_t frames_since_gain_change_ = 0;
  bool store_image_data_;

  std::optional<joint_model::JointProperties> updated_properties_;

  struct {
    std::map<joint_model::JointModelErrorCode, prometheus::Counter*> image_errors;
    std::map<uint64_t, prometheus::Counter*> image;
    prometheus::Histogram* image_processing_time;
    prometheus::Gauge* image_consecutive_errors;
  } metrics_;

  std::function<void(std::function<void()>)> post_;
};

class ScannerOutputCBImpl : public ScannerOutputCB {
 public:
  void ScannerOutput(const common::Groove& groove,
                     const std::array<common::Point, joint_model::INTERPOLATED_SNAKE_SIZE>& profile,
                     uint64_t time_stamp, slice_provider::SliceConfidence confidence) override {};
};

class ScannerExposed : public ScannerImpl {
 public:
  ScannerExposed(image_provider::ImageProvider* image_provider, joint_model::JointModelPtr joint_model,
                 slice_provider::SliceProviderPtr slice_provider, const LaserCallback& laser_toggle,
                 ScannerOutputCBImpl* scanner_ouput_cb, image_logger::ImageLogger* image_logger,
                 prometheus::Registry* registry)
      : ScannerImpl(image_provider, std::move(slice_provider), laser_toggle, scanner_ouput_cb, std::move(joint_model),
                    image_logger, registry) {}

  using ScannerImpl::Update;
};

}  // namespace scanner

namespace std {
template <>
struct is_error_code_enum<scanner::ScannerErrorCode> : true_type {};
}  // namespace std
