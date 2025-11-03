#pragma once

#include <boost/outcome.hpp>
#include <boost/thread/thread.hpp>
#include <filesystem>
#include <memory>
#include <optional>
#include <tuple>
#include <vector>

#include "scanner/image/image.h"
#include "scanner/image_provider/image_provider.h"
#include "scanner/image_provider/image_provider_configuration.h"

namespace scanner::image_provider {

enum class CameraSimulationErrorCode : uint32_t {
  NO_ERROR                    = 0,
  NO_IMAGES_FOUND             = 1,
  IMAGE_EVENT_HANDLER_NOT_SET = 2,
};

// NOLINTNEXTLINE(*-identifier-naming)
[[maybe_unused]] auto make_error_code(CameraSimulationErrorCode) -> std::error_code;

class CameraSimulation : public ImageProvider {
 public:
  explicit CameraSimulation(const SimConfig&, bool loop);
  CameraSimulation(CameraSimulation&)                     = delete;
  auto operator=(CameraSimulation&) -> CameraSimulation&  = delete;
  CameraSimulation(CameraSimulation&&)                    = delete;
  auto operator=(CameraSimulation&&) -> CameraSimulation& = delete;

  ~CameraSimulation() override;

  auto Start(enum scanner::ScannerSensitivity sensitivity) -> boost::outcome_v2::result<void> override;
  void Stop() override;
  auto Started() const -> bool override;
  void ResetFOVAndGain() override;
  void AdjustGain(double) override;
  void SetVerticalFOV(int offset_from_top, int height) override;
  void SetHorizontalFOV(int offset_from_left, int width) override;
  void SetFOV(std::optional<std::tuple<int, int>> vertical,
              std::optional<std::tuple<int, int>> horizontal) override {
    if (vertical.has_value()) {
      offset_ = std::get<0>(vertical.value());
      height_ = std::get<1>(vertical.value());
    }

    if (horizontal.has_value()) {
      offset_x_ = std::get<0>(horizontal.value());
      width_    = std::get<1>(horizontal.value());
    }
  }
  auto GetVerticalFOVOffset() -> int override;
  auto GetVerticalFOVHeight() -> int override;
  auto GetHorizontalFOVOffset() -> int override;
  auto GetHorizontalFOVWidth() -> int override;
  auto GetMaxHorizontalWidth() -> int override;
  auto GetSerialNumber() -> std::string override;
  void SetOnImage(OnImage on_image) override { on_image_ = on_image; };

 private:
  void Run();
  auto GetImage() -> std::tuple<std::optional<std::unique_ptr<image::Image>>, std::optional<uint32_t>>;

  boost::thread grabbing_thread_;
  bool started_;
  std::vector<std::filesystem::path> image_files_;
  size_t current_image_ = 0;
  bool real_time_mode_;
  bool loop_;
  uint32_t previous_time_stamp_;

  int offset_;
  int height_;
  int offset_x_;
  int width_;
  int max_width_;
  std::string serial_number_;
  OnImage on_image_;
};

}  // namespace scanner::image_provider

namespace std {
template <>
struct is_error_code_enum<scanner::image_provider::CameraSimulationErrorCode> : true_type {};
}  // namespace std
