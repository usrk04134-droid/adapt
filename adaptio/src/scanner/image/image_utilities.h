#pragma once

#include <optional>

#include "scanner/image/image_types.h"

namespace scanner::image {

class ImageUtility {
 public:
  static auto CropImage(const RawImageData& src_image, int start_row, int start_col, int rows, int cols)
      -> std::optional<const RawImageData>;
};

}  // namespace scanner::image
