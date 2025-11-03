#pragma once

#include <boost/outcome.hpp>
#include <functional>
#include <memory>
#include <optional>
#include <tuple>
#include <utility>

#include "scanner/core/scanner_types.h"
#include "scanner/image/image.h"

namespace scanner::image_provider {

class ImageProvider {
 public:
  using OnImage = std::function<void(std::unique_ptr<image::Image>)>;

  ImageProvider() {};

  ImageProvider(ImageProvider&)                     = delete;
  auto operator=(ImageProvider&) -> ImageProvider&  = delete;
  ImageProvider(ImageProvider&&)                    = delete;
  auto operator=(ImageProvider&&) -> ImageProvider& = delete;

  virtual ~ImageProvider() = default;

  virtual void SetOnImage(OnImage on_image) = 0;

  virtual auto Start(enum scanner::ScannerSensitivity sensitivity) -> boost::outcome_v2::result<void> = 0;
  virtual void Stop()                                                                                 = 0;
  virtual auto Started() const -> bool                                                                = 0;

  virtual void ResetFOVAndGain()                                 = 0;
  virtual void SetVerticalFOV(int offset_from_top, int height)   = 0;
  virtual void SetHorizontalFOV(int offset_from_left, int width) = 0;
  virtual void SetFov(std::optional<std::tuple<int, int>> vertical,
                      std::optional<std::tuple<int, int>> horizontal) {
    if (vertical) {
      auto [offset, height] = *vertical;
      SetVerticalFOV(offset, height);
    }
    if (horizontal) {
      auto [offset, width] = *horizontal;
      SetHorizontalFOV(offset, width);
    }
  }
  virtual void AdjustGain(double factor)                         = 0;
  virtual auto GetVerticalFOVOffset() -> int                     = 0;
  virtual auto GetVerticalFOVHeight() -> int                     = 0;
  virtual auto GetHorizontalFOVOffset() -> int                   = 0;
  virtual auto GetHorizontalFOVWidth() -> int                    = 0;
  virtual auto GetMaxHorizontalWidth() -> int                    = 0;
  virtual auto GetSerialNumber() -> std::string                  = 0;
};

using ImageProviderPtr = std::unique_ptr<ImageProvider>;

}  // namespace scanner::image_provider
