#pragma once

#include <optional>

#include "core/image/image_types.h"

namespace core::image {

class ImageUtility {
 public:
  static auto CropImage(const core::image::RawImageData& src_image, int start_row, int start_col, int rows, int cols)
      -> std::optional<const core::image::RawImageData>;
};

}  // namespace core::image
