#include "image_utilities.h"

#include <Eigen/Core>
#include <optional>

#include "core/image/image_types.h"
#include "core/logging/application_log.h"

using core::image::ImageUtility;

auto ImageUtility::CropImage(const core::image::RawImageData& src_image, int start_row, int start_col, int rows,
                             int cols) -> std::optional<const core::image::RawImageData> {
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

#ifndef DOCTEST_CONFIG_DISABLE

// NOLINTBEGIN(*-magic-numbers)

#include <doctest/doctest.h>

#include <cstdint>

using core::image::RawImageData;

TEST_SUITE("Image_utility") {
  TEST_CASE("Test image crop") {
    Eigen::Matrix<std::uint8_t, 3, 3> img{
        {2, 3, 4 },
        {5, 6, 7 },
        {8, 9, 10},
    };
    auto maybe_image = core::image::ImageUtility::CropImage(img, 1, 1, 2, 2);
    CHECK_EQ(maybe_image.has_value(), true);
    CHECK_EQ(maybe_image.value(), img.block(1, 1, 2, 2));
  }
  TEST_CASE("Test image crop - start outside") {
    Eigen::Matrix<std::uint8_t, 3, 3> img{
        {2, 3, 4 },
        {5, 6, 7 },
        {8, 9, 10},
    };
    auto maybe_image = core::image::ImageUtility::CropImage(img, 3, 3, 2, 2);
    CHECK_EQ(maybe_image.has_value(), false);
  }
  TEST_CASE("Test image crop - too few cols") {
    Eigen::Matrix<std::uint8_t, 3, 3> img{
        {2, 3, 4 },
        {5, 6, 7 },
        {8, 9, 10},
    };
    auto maybe_image = core::image::ImageUtility::CropImage(img, 1, 1, 2, 1);
    CHECK_EQ(maybe_image.has_value(), false);
  }

  TEST_CASE("Test image crop - outside truncated") {
    Eigen::Matrix<std::uint8_t, 3, 3> img{
        {2, 3, 4 },
        {5, 6, 7 },
        {8, 9, 10},
    };
    auto maybe_image = core::image::ImageUtility::CropImage(img, 1, 1, 3, 3);
    CHECK_EQ(maybe_image.has_value(), true);
    CHECK_EQ(maybe_image.value().cols(), 2);
    CHECK_EQ(maybe_image.value().rows(), 2);
    CHECK_EQ(maybe_image.value(), img.block(1, 1, 2, 2));
  }
}

// NOLINTEND(*-magic-numbers)
#endif
