#include "scanner/image/image_utilities.h"

#include <Eigen/Core>
#include <optional>

#include "common/logging/application_log.h"
#include "scanner/image/image_types.h"

namespace scanner::image {

auto ImageUtility::CropImage(const RawImageData& src_image, int start_row, int start_col, int rows, int cols)
    -> std::optional<const RawImageData> {
  // Check if start point is outside of image
  if (start_row >= (src_image.rows() - 1) || start_col >= (src_image.cols() - 1)) {
    LOG_ERROR("Start position of crop is outside image: {},{} {},{}", start_row, start_col, src_image.rows(),
              src_image.cols());
    return std::nullopt;
  }

  // Check if valid size on number of cols and rows to crop
  if (rows <= 1 || cols <= 1) {
    LOG_ERROR("Try to crop too few rows/cols {}/{}", rows, cols);
    return std::nullopt;
  }

  // Make sure to not crop outside of image
  // Truncate rows and cols if needed
  if ((start_row + rows) > src_image.rows()) {
    rows = src_image.rows() - start_row;
  }

  if ((start_col + cols) > src_image.cols()) {
    cols = src_image.cols() - start_col;
  }

  return src_image.block(start_row, start_col, rows, cols);
}
}  // namespace scanner::image
