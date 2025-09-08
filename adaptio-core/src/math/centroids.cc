#ifndef DOCTEST_CONFIG_DISABLE
// NOLINTBEGIN(*-magic-numbers)

#include "core/math/centroids.h"

#include <doctest/doctest.h>

#include <cstdint>
#include <Eigen/Core>

#include "core/image/camera_model.h"
#include "core/image/image_types.h"

using Eigen::Index;
using Eigen::Matrix3d;
using Eigen::RowVectorX;
using Eigen::RowVectorXd;
using Eigen::Vector3d;

using core::image::CameraProperties;
using core::image::RawImageData;
using core::math::GetCentroidsByColumn;
using core::math::GetCentroidsByRow;

TEST_SUITE("Image utilities tests") {
  TEST_CASE("GetCentroidByColumn") {
    // clang-format off
    auto data = new uint8_t[30]{
      0, 0, 0,  1,  2,  3,  3, 3, 4, 5, 
      6, 7, 11, 20, 20, 10, 7, 2, 1, 1,
      1, 1, 0,  0,  0,  0,  0, 0, 0, 0};
    // clang-format on
    int height = 30, width = 1;

    auto matrix = Eigen::Map<RawImageData>(data, height, width);

    CHECK_EQ(core::math::COG_WIDTH, 13);  // If this is changed the test will need to be recalculated:

    // import numpy as np
    // m = np.array([ 0, 0, 0,  1,  2,  3,  3, 3, 4, 5,6, 7, 11, 20, 20, 10, 7, 2, 1, 1,1, 1, 0,  0,  0,  0,  0, 0, 0,
    // 0]) half = int((COG_WIDTH - 1) / 2) top = np.argmax(m) - half  # Reverse if "Reversed" x = m[top:(top +
    // COG_WIDTH)] answer = np.dot(x, np.linspace(0, len(x) - 1, len(x))) / np.sum(x)

    {
      auto _centroids = GetCentroidsByColumn(matrix, core::math::CentroidSearchDirection::Normal, 0);

      CHECK_EQ(_centroids.has_value(), true);

      auto centroids = _centroids.value();

      CHECK_EQ(centroids.rows(), 2);
      CHECK_EQ(centroids.cols(), 1);

      CHECK_LE(centroids(1, 0), 12.8f);
      CHECK_GE(centroids(1, 0), 12.7f);
      CHECK_EQ(centroids(0, 0), 0.0f);
    }

    {
      auto _centroids = GetCentroidsByColumn(matrix, core::math::CentroidSearchDirection::Reversed, 0);

      CHECK_EQ(_centroids.has_value(), true);

      auto centroids = _centroids.value();

      CHECK_EQ(centroids.rows(), 2);
      CHECK_EQ(centroids.cols(), 1);

      CHECK_LE(centroids(1, 0), 13.1f);
      CHECK_GE(centroids(1, 0), 13.0f);
      CHECK_EQ(centroids(0, 0), 0.0f);
    }

    delete[] data;
  }

  TEST_CASE("GetCentroidByColumnEmpty") {
    // clang-format off
    auto data = new uint8_t[15]{0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0};
    // clang-format on
    int height = 3, width = 5;

    auto matrix = Eigen::Map<RawImageData>(data, height, width);

    auto centroids = GetCentroidsByColumn(matrix, core::math::CentroidSearchDirection::Normal, 0);

    CHECK_EQ(centroids.has_value(), false);

    centroids = GetCentroidsByColumn(matrix, core::math::CentroidSearchDirection::Reversed, 0);

    CHECK_EQ(centroids.has_value(), false);

    delete[] data;
  }

  TEST_CASE("GetCentroidByRow") {
    // clang-format off
    auto data = new uint8_t[30]{
      0, 0, 0,  1,  2,  3,  3, 3, 4, 5, 
      6, 7, 11, 20, 20, 10, 7, 2, 1, 1,
      1, 1, 0,  0,  0,  0,  0, 0, 0, 0};
    // clang-format on
    int height = 1, width = 30;

    auto matrix = Eigen::Map<RawImageData>(data, height, width);

    CHECK_EQ(core::math::COG_WIDTH, 13);  // If this is changed the test will need to be recalculated:

    {
      auto _centroids = GetCentroidsByRow(matrix, core::math::CentroidSearchDirection::Normal, 0);

      CHECK_EQ(_centroids.has_value(), true);

      auto centroids = _centroids.value();

      CHECK_EQ(centroids.rows(), 2);
      CHECK_EQ(centroids.cols(), 1);

      CHECK_LE(centroids(0, 0), 12.8f);
      CHECK_GE(centroids(0, 0), 12.7f);
      CHECK_EQ(centroids(1, 0), 0.0f);
    }

    {
      auto _centroids = GetCentroidsByRow(matrix, core::math::CentroidSearchDirection::Reversed, 0);

      CHECK_EQ(_centroids.has_value(), true);

      auto centroids = _centroids.value();

      CHECK_EQ(centroids.rows(), 2);
      CHECK_EQ(centroids.cols(), 1);

      CHECK_LE(centroids(0, 0), 13.1f);
      CHECK_GE(centroids(0, 0), 13.0f);
      CHECK_EQ(centroids(1, 0), 0.0f);
    }

    delete[] data;
  }

  TEST_CASE("GetCentroidByRowEmpty") {
    // clang-format off
    auto data = new uint8_t[15]{0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0};
    // clang-format on
    int height = 3, width = 5;

    auto matrix = Eigen::Map<RawImageData>(data, height, width);

    auto centroids = GetCentroidsByRow(matrix, core::math::CentroidSearchDirection::Normal, 0);

    CHECK_EQ(centroids.has_value(), false);

    centroids = GetCentroidsByRow(matrix, core::math::CentroidSearchDirection::Reversed, 0);

    CHECK_EQ(centroids.has_value(), false);

    delete[] data;
  }
}

// NOLINTEND(*-magic-numbers)
#endif
