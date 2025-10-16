#pragma once

#include <boost/outcome.hpp>
#include <functional>
#include <memory>
#include <optional>
#include <utility>

#include "core/image/image.h"
#include "core/scanner/scanner_types.h"

namespace core::scanner {

class ImageProvider {
 public:
  using OnImage = std::function<void(std::unique_ptr<core::image::Image>)>;

  ImageProvider() {};

  ImageProvider(ImageProvider&)                     = delete;
  auto operator=(ImageProvider&) -> ImageProvider&  = delete;
  ImageProvider(ImageProvider&&)                    = delete;
  auto operator=(ImageProvider&&) -> ImageProvider& = delete;

  virtual ~ImageProvider() = default;

  virtual void SetOnImage(OnImage on_image) = 0;

  virtual auto Start(enum ScannerSensitivity sensitivity) -> boost::outcome_v2::result<void> = 0;
  virtual void Stop()                                                                        = 0;
  [[nodiscard]] virtual auto Started() const -> bool                                         = 0;

  virtual void ResetFOVAndGain()                               = 0;
  virtual void SetVerticalFOV(int offset_from_top, int height) = 0;
  // Dynamically adjust horizontal ROI (offset from left and width)
  // Default: no-op for providers that don't support horizontal ROI
  virtual void SetHorizontalFOV(int offset_from_left, int width) {};
  virtual void AdjustGain(double factor)                       = 0;
  virtual auto GetVerticalFOVOffset() -> int                   = 0;
  virtual auto GetVerticalFOVHeight() -> int                   = 0;
  // Query current horizontal ROI relative to configured FOV
  // Default: return 0/0 meaning "unknown/unsupported"
  virtual auto GetHorizontalFOVOffset() -> int { return 0; };
  virtual auto GetHorizontalFOVWidth() -> int { return 0; };
  virtual auto GetSerialNumber() -> std::string                = 0;
};

using ImageProviderPtr = std::unique_ptr<ImageProvider>;

}  // namespace core::scanner
