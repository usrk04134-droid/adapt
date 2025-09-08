#include <doctest/doctest.h>

#include <cstdint>

#include "scanner/image/image_builder.h"
#include "scanner/image/image_types.h"

// NOLINTBEGIN(*-magic-numbers)
namespace scanner::image {

TEST_SUITE("Image") {
  TEST_CASE("Test image pixel filter function") {
    auto data  = new uint8_t[4]{8, 65, 51, 43};
    int height = 2, width = 2;

    auto matrix = Eigen::Map<RawImageData>(data, height, width);

    auto image = ImageBuilder::From(matrix, 0).Finalize().value();

    image->PixelFilter<uint8_t>([](uint8_t pixel_value, Eigen::Index row, Eigen::Index column) -> uint8_t {
      if (pixel_value < 50) {
        return 0;
      }

      return pixel_value;
    });

    CHECK_EQ((image->Data())(0, 0), 0);
    CHECK_EQ((image->Data())(0, 1), 65);
    CHECK_EQ((image->Data())(1, 0), 51);
    CHECK_EQ((image->Data())(1, 1), 0);

    image->PixelFilter<uint8_t>([](uint8_t pixel_value, Eigen::Index row, Eigen::Index column) -> uint8_t {
      if (row < 1) {
        return pixel_value;
      }

      return 0;
    });

    CHECK_EQ((image->Data())(0, 0), 0);
    CHECK_EQ((image->Data())(0, 1), 65);
    CHECK_EQ((image->Data())(1, 0), 0);
    CHECK_EQ((image->Data())(1, 1), 0);

    delete[] data;
  }
}

}  // namespace scanner::image
// NOLINTEND(*-magic-numbers)
