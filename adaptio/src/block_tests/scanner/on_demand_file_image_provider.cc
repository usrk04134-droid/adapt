#include "on_demand_file_image_provider.h"

#include <cassert>
#include <chrono>
#include <filesystem>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <string>
#include <utility>

#include "common/logging/application_log.h"
#include "scanner/core/scanner_types.h"
#include "scanner/image/image.h"
#include "scanner/image/image_builder.h"
#include "scanner/image/tiff_handler_impl.h"

using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

OnDemandFileImageProvider::OnDemandFileImageProvider(const std::string& serial_number,
                                                     const std::filesystem::path& base_dir)
    : serial_(serial_number), image_dir_(base_dir / ("sn_" + serial_)) {}

auto OnDemandFileImageProvider::SetOnImage(OnImage on_image) -> void { on_image_ = std::move(on_image); }

auto OnDemandFileImageProvider::Start(enum scanner::ScannerSensitivity /*sensitivity*/)
    -> boost::outcome_v2::result<void> {
  started_ = true;
  return boost::outcome_v2::success();
}

auto OnDemandFileImageProvider::Stop() -> void { started_ = false; }

auto OnDemandFileImageProvider::Started() const -> bool { return started_; }

auto OnDemandFileImageProvider::ResetFOVAndGain() -> void { SetVerticalFOV(0, 0); }

auto OnDemandFileImageProvider::SetVerticalFOV(int offset_from_top, int height) -> void {
  fov_offset_ = offset_from_top;
  fov_height_ = height;
}

auto OnDemandFileImageProvider::AdjustGain(double /*factor*/) -> void {}

auto OnDemandFileImageProvider::GetVerticalFOVOffset() -> int { return fov_offset_; }

auto OnDemandFileImageProvider::GetVerticalFOVHeight() -> int { return fov_height_; }

auto OnDemandFileImageProvider::GetSerialNumber() -> std::string { return serial_; }

auto OnDemandFileImageProvider::Dispatch(const std::string& relative_path) -> bool {
  assert(started_ && on_image_);

  const auto full = image_dir_ / relative_path;
  auto maybe_img  = Load(full);
  if (!maybe_img.has_value()) {
    LOG_ERROR("OnDemandFileImageProvider failed to load image: {}", full.string());
    return false;
  }

  auto img = std::move(*maybe_img);
  img->SetTimestamp(high_resolution_clock::now() - milliseconds(90));
  on_image_(std::move(img));
  return true;
}

auto OnDemandFileImageProvider::Load(const std::filesystem::path& file_path) -> std::optional<ImagePtr> {
  auto mat = cv::imread(file_path.string(), cv::IMREAD_GRAYSCALE);
  if (mat.empty()) {
    LOG_ERROR("OnDemandFileImageProvider: imread failed for {}", file_path.string());
    return std::nullopt;
  }

  // Optional metadata (kept for parity with existing helpers)
  (void)scanner::image::ReadTimestamp(file_path);
  auto maybe_fov_xy            = scanner::image::ReadFovOffset(file_path);
  const int vertical_crop_from = maybe_fov_xy.has_value() ? static_cast<int>(std::get<1>(*maybe_fov_xy)) : 0;

  auto maybe_image = scanner::image::ImageBuilder::From(mat, file_path.filename(), vertical_crop_from).Finalize();
  if (!maybe_image.has_value()) {
    LOG_ERROR("OnDemandFileImageProvider: ImageBuilder::Finalize failed for {}", file_path.string());
    return std::nullopt;
  }

  return std::move(maybe_image.value());
}
