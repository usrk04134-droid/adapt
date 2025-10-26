#pragma once

#include <boost/outcome.hpp>
#include <boost/thread/thread.hpp>
#include <filesystem>
#include <memory>
#include <vector>

#include "core/scanner/image_provider.h"
#include "core/scanner/image_provider_configuration.h"

namespace core::scanner {

enum class CameraSimulationErrorCode : uint32_t {
  NO_ERROR                    = 0,
  NO_IMAGES_FOUND             = 1,
  IMAGE_EVENT_HANDLER_NOT_SET = 2,
};

// NOLINTNEXTLINE(*-identifier-naming)
[[maybe_unused]] auto make_error_code(CameraSimulationErrorCode) -> std::error_code;

class CameraSimulation : public core::scanner::ImageProvider {
 public:
  explicit CameraSimulation(const SimConfig&, bool loop);
  CameraSimulation(CameraSimulation&)                     = delete;
  auto operator=(CameraSimulation&) -> CameraSimulation&  = delete;
  CameraSimulation(CameraSimulation&&)                    = delete;
  auto operator=(CameraSimulation&&) -> CameraSimulation& = delete;

  ~CameraSimulation() override;

  auto Start(enum ScannerSensitivity sensitivity) -> boost::outcome_v2::result<void> override;
  void Stop() override;
  [[nodiscard]] auto Started() const -> bool override;
  void ResetFOVAndGain() override;
  void AdjustGain(double) override;
  void SetVerticalFOV(int offset_from_top, int height) override;
  auto GetVerticalFOVOffset() -> int override;
  auto GetVerticalFOVHeight() -> int override;
  // Horizontal ROI control (no-op in simulation)
  void SetHorizontalFOV(int offset_from_left, int width) override {}
  auto GetHorizontalFOVOffset() -> int override { return 0; }
  auto GetHorizontalFOVWidth() -> int override { return 0; }
  auto GetSerialNumber() -> std::string override;
  void SetOnImage(OnImage on_image) override { on_image_ = on_image; };

 private:
  void Run();
  auto GetImage() -> std::tuple<std::optional<std::unique_ptr<core::image::Image>>, std::optional<uint32_t>>;

  boost::thread grabbing_thread_;
  bool started_;
  std::vector<std::filesystem::path> image_files_;
  size_t current_image_ = 0;
  bool real_time_mode_;
  bool loop_;
  uint32_t previous_time_stamp_;

  int offset_;
  int height_;
  std::string serial_number_;
  OnImage on_image_;
};

}  // namespace core::scanner

namespace std {
template <>
struct is_error_code_enum<core::scanner::CameraSimulationErrorCode> : true_type {};
}  // namespace std
