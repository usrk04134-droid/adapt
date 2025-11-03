#pragma once

#include <prometheus/gauge.h>
#include <prometheus/registry.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/PylonIncludes.h>

#include <boost/outcome.hpp>
#include <map>
#include <optional>
#include <thread>
#include <tuple>

#include "scanner/image/image.h"
#include "scanner/image_provider/image_provider.h"
#include "scanner/image_provider/image_provider_configuration.h"
#include "scanner/image_provider/src/buffered_channel.h"

namespace scanner::image_provider {

// Receives images from Pylon and passes them to the scanner
class BaslerCameraUpstreamImageEventHandler : public Pylon::CImageEventHandler {
 public:
  using Timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>;

  BaslerCameraUpstreamImageEventHandler(ImageProvider::OnImage on_image, Timestamp start_time, int64_t start_tick,
                                        int original_offset, int original_offset_x);

  void OnImageGrabbed(Pylon::CInstantCamera &camera, const Pylon::CGrabResultPtr &) override;
  void OnImagesSkipped(Pylon::CInstantCamera &camera, size_t) override;

 private:
  Timestamp base_timestamp;
  int64_t base_tick;
  int original_offset_;
  int original_offset_x_;
  ImageProvider::OnImage on_image_;
};

enum class BaslerCameraErrorCode : uint32_t {
  NO_ERROR                    = 0,
  NO_CAMERA_FOUND             = 1,
  FAILED_TO_ACQUIRE_CAMERA    = 2,
  FAILED_TO_SET_CONFIG        = 3,
  FAILED_TO_OPEN_CAMERA       = 4,
  IMAGE_EVENT_HANDLER_NOT_SET = 5,
};

// NOLINTNEXTLINE(*-identifier-naming)
[[maybe_unused]] auto make_error_code(BaslerCameraErrorCode) -> std::error_code;

class BaslerCamera : public ImageProvider {
 public:
  explicit BaslerCamera(const BaslerConfig &config, const Fov &fov, prometheus::Registry *registry);

  BaslerCamera(BaslerCamera &)                      = delete;
  auto operator=(BaslerCamera &) -> BaslerCamera &  = delete;
  BaslerCamera(BaslerCamera &&)                     = delete;
  auto operator=(BaslerCamera &&) -> BaslerCamera & = delete;

  ~BaslerCamera() override;

  auto Start(enum scanner::ScannerSensitivity sensitivity) -> boost::outcome_v2::result<void> override;
  void Stop() override;
  auto Started() const -> bool override;
  void ResetFOVAndGain() override;
  void SetVerticalFOV(int offset_from_top, int height) override;
  void SetHorizontalFOV(int offset_from_left, int width) override;
  void SetFOV(std::optional<std::tuple<int, int>> vertical,
              std::optional<std::tuple<int, int>> horizontal) override;
  void AdjustGain(double factor) override;
  auto GetVerticalFOVOffset() -> int override;
  auto GetVerticalFOVHeight() -> int override;
  auto GetHorizontalFOVOffset() -> int override;
  auto GetHorizontalFOVWidth() -> int override;
  auto GetMaxHorizontalWidth() -> int override;
  auto GetSerialNumber() -> std::string override;
  void SetOnImage(OnImage on_image) override { on_image_ = on_image; };

 private:
  auto InitializeCamera(float gain, float exposure_time) -> boost::outcome_v2::result<void>;
  auto StartCamera() -> boost::outcome_v2::result<void>;
  void SetupMetrics(prometheus::Registry *registry);
  void UpdateMetrics();

  std::unique_ptr<Pylon::CBaslerUniversalInstantCamera> camera_;
  BufferedChannel<image::ImagePtr>::WriterPtr channel_;
  std::thread grabbing_thread_;
  BaslerConfig config_;
  Fov fov_;
  //int vertical_crop_offset_  = 0;
  //int horizontal_crop_offset_ = 0;
  OnImage on_image_;

  double initial_gain_ = 0.0;

  struct {
    std::map<std::string, prometheus::Gauge &> temperature_status_gauges;
    prometheus::Gauge *temperature_status_error{};
    prometheus::Gauge *temperature{};
    prometheus::Gauge *max_temperature{};
  } metrics_;
  std::chrono::steady_clock::time_point last_get_scanner_metrics_;
};

}  // namespace scanner::image_provider

namespace std {
template <>
struct is_error_code_enum<scanner::image_provider::BaslerCameraErrorCode> : true_type {};
}  // namespace std
