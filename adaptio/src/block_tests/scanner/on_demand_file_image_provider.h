#pragma once

#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "scanner/image/image.h"
#include "scanner/image_provider/image_provider.h"

class OnDemandFileImageProvider : public scanner::image_provider::ImageProvider {
 public:
  explicit OnDemandFileImageProvider(const std::string& serial_number, const std::filesystem::path& base_dir);

  // ImageProvider
  auto SetOnImage(OnImage on_image) -> void override;
  auto Start(enum scanner::ScannerSensitivity /*sensitivity*/) -> boost::outcome_v2::result<void> override;
  auto Stop() -> void override;
  auto Started() const -> bool override;

  auto ResetFOVAndGain() -> void override;
  auto SetVerticalFOV(int offset_from_top, int height) -> void override;
  auto AdjustGain(double /*factor*/) -> void override;
  auto GetVerticalFOVOffset() -> int override;
  auto GetVerticalFOVHeight() -> int override;

  auto GetSerialNumber() -> std::string override;

  auto Dispatch(const std::string& relative_path) -> bool;

 private:
  using ImagePtr = std::unique_ptr<scanner::image::Image>;
  auto Load(const std::filesystem::path& file_path) -> std::optional<ImagePtr>;

  std::string serial_;
  std::filesystem::path image_dir_;
  OnImage on_image_;
  bool started_{false};

  int fov_offset_{0};
  int fov_height_{0};
};
