#include <cmath>
#include <cstdlib>
#include <opencv2/imgcodecs.hpp>
#include <optional>

#include "scanner/image/camera_model.h"
#include "scanner/image/image_builder.h"
#ifndef DOCTEST_CONFIG_DISABLE
#include <doctest/doctest.h>

#include "scanner/joint_model/src/snake.h"
using Eigen::Index;
using Eigen::RowVectorXd;
using scanner::image::WorkspaceCoordinates;

TEST_SUITE("Test Snake") {
  TEST_CASE("FromImage - close to edge") {
    auto grayscale_image =
        cv::imread("./src/scanner/joint_model/test/test_data/1755001276997.tiff", cv::IMREAD_GRAYSCALE);
    auto maybe_image = scanner::image::ImageBuilder::From(grayscale_image, "1755001276997.tiff", 0).Finalize();
    auto *image      = maybe_image.value().get();
    auto res         = scanner::joint_model::Snake::FromImage(*image, {}, 16);
    CHECK(res.has_value());
  }
  TEST_CASE("FromImage - vertically cropped image") {
    auto grayscale_image =
        cv::imread("./src/scanner/joint_model/test/test_data/1755001276997.tiff", cv::IMREAD_GRAYSCALE);
    auto maybe_image = scanner::image::ImageBuilder::From(grayscale_image, "1755001276997.tiff", 0).Finalize();
    auto *image      = maybe_image.value().get();
    image->SetHorizontalCrop(300, 3200);
    auto res = scanner::joint_model::Snake::FromImage(*image, {}, 16);
    CHECK(res.has_value());
  }
  TEST_CASE("FromImage - black image") {
    // Should not get any snake and should not crash
    auto grayscale_image =
        cv::imread("./src/scanner/joint_model/test/test_data/1754561083373.tiff", cv::IMREAD_GRAYSCALE);
    auto maybe_image = scanner::image::ImageBuilder::From(grayscale_image, "1754561083373.tiff", 0).Finalize();
    auto *image      = maybe_image.value().get();
    auto res         = scanner::joint_model::Snake::FromImage(*image, {}, 16);
    CHECK(!res.has_value());
  }
}
#endif
