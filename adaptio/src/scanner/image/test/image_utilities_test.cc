
#include "scanner/image/image_utilities.h"

#include <doctest/doctest.h>

#include <cstdint>
#include <Eigen/Core>

#include "scanner/image/image_types.h"

// NOLINTBEGIN(*-magic-numbers)
namespace scanner::image {

TEST_SUITE("Image_utility") {
  TEST_CASE("Test image crop") {
    Eigen::Matrix<std::uint8_t, 3, 3> img{
        {2, 3, 4 },
        {5, 6, 7 },
        {8, 9, 10},
    };
    auto maybe_image = ImageUtility::CropImage(img, 1, 1, 2, 2);
    CHECK_EQ(maybe_image.has_value(), true);
    CHECK_EQ(maybe_image.value(), img.block(1, 1, 2, 2));
  }
  TEST_CASE("Test image crop - start outside") {
    Eigen::Matrix<std::uint8_t, 3, 3> img{
        {2, 3, 4 },
        {5, 6, 7 },
        {8, 9, 10},
    };
    auto maybe_image = ImageUtility::CropImage(img, 3, 3, 2, 2);
    CHECK_EQ(maybe_image.has_value(), false);
  }
  TEST_CASE("Test image crop - too few cols") {
    Eigen::Matrix<std::uint8_t, 3, 3> img{
        {2, 3, 4 },
        {5, 6, 7 },
        {8, 9, 10},
    };
    auto maybe_image = ImageUtility::CropImage(img, 1, 1, 2, 1);
    CHECK_EQ(maybe_image.has_value(), false);
  }

  TEST_CASE("Test image crop - outside truncated") {
    Eigen::Matrix<std::uint8_t, 3, 3> img{
        {2, 3, 4 },
        {5, 6, 7 },
        {8, 9, 10},
    };
    auto maybe_image = ImageUtility::CropImage(img, 1, 1, 3, 3);
    CHECK_EQ(maybe_image.has_value(), true);
    CHECK_EQ(maybe_image.value().cols(), 2);
    CHECK_EQ(maybe_image.value().rows(), 2);
    CHECK_EQ(maybe_image.value(), img.block(1, 1, 2, 2));
  }
}

}  // namespace scanner::image
// NOLINTEND(*-magic-numbers)
