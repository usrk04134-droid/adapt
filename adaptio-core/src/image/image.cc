#include "core/image/image.h"

#include <boost/uuid/uuid.hpp>
#include <chrono>
#include <cstdint>
#include <Eigen/Core>
#include <string>
#include <utility>
#include <vector>

#include "core/image/image_types.h"

using core::image::Image;
using Eigen::Index;
using Eigen::last;
using Eigen::Matrix3d;
using Eigen::RowVectorXd;

Image::Image(RawImageData matrix)
    : data_(std::move(matrix)),
      uuid_(boost::uuids::random_generator()()),
      timestamp_(std::chrono::high_resolution_clock::now()) {}

Image::Image(RawImageData matrix, const std::string& img_name)
    : data_(std::move(matrix)),
      uuid_(boost::uuids::random_generator()()),
      timestamp_(std::chrono::high_resolution_clock::now()),
      img_name_(img_name) {}

Image::Image(RawImageData matrix, Timestamp timestamp)
    : data_(std::move(matrix)), uuid_(boost::uuids::random_generator()()), timestamp_(timestamp) {}

Image::~Image() = default;

auto Image::Data() const -> const RawImageData& { return data_; }

auto Image::AsBytes() -> std::vector<uint8_t> { return {}; }

auto Image::GetUuid() -> boost::uuids::uuid { return uuid_; }

auto Image::GetTimestamp() const -> Timestamp { return timestamp_; }
void Image::SetTimestamp(Timestamp t) { timestamp_ = t; }

auto Image::GetImageName() const -> std::string { return img_name_; }

/*
void Image::Show() {
  cv::Mat cv_image;
  cv::eigen2cv(static_cast<Eigen::Matrix<uint8_t, -1, -1, 1>>(Data()), cv_image);
  LOG_TRACE("OpenCV Mat size: {}x{}", cv_image.cols, cv_image.rows);
  cv::namedWindow("output", cv::WINDOW_NORMAL);
  cv::imshow("output", cv_image);
  cv::waitKey();
}

auto Image::Process(const JointProperties& joint_properties) -> boost::outcome_v2::result<std::unique_ptr<ImageData>> {
  LOG_TRACE("Image processing started for {}", boost::uuids::to_string(uuid_).c_str());

  MatrixType matrix = data_;
  auto camera_properties = camera_properties_;
  auto image_data = std::make_unique<ImageData>();

  const Eigen::Matrix3d H =
      CalculateTiltTransformationMatrix(camera_properties.intrinsic.rho, camera_properties.intrinsic.tau,
                                        camera_properties.intrinsic.m, camera_properties.intrinsic.d);

  auto start = std::chrono::high_resolution_clock::now();

  PixelFilter<uint8_t>(matrix, [=](uint8_t pixel_value, Index row, Index column) -> uint8_t {
    if (pixel_value < 55) {
      return 0;
    }

    return pixel_value;
  });

  auto [x_centroids, y_centroids] = GetCentroidsByColumn(matrix);

  auto [x_wcs, y_wcs] = IcsToWcs(x_centroids, y_centroids, H, camera_properties);

  auto [left_surface, right_surface] = SurfaceLines(x_wcs, y_wcs, joint_properties);

  auto [left_groove_wall, right_groove_wall] = FindGrooveLines(x_wcs, y_wcs, joint_properties);

  auto diff = std::chrono::high_resolution_clock::now() - start;
  LOG_INFO("Processed image in {}ms", std::chrono::duration_cast<std::chrono::milliseconds>(diff).count());

  return image_data;
}
*/

#ifndef DOCTEST_CONFIG_DISABLE
// NOLINTBEGIN(*-magic-numbers)

#include <doctest/doctest.h>

#include "core/image/image_builder.h"

using core::image::ImageBuilder;
using core::image::RawImageData;

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

// NOLINTEND(*-magic-numbers)
#endif