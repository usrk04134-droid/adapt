#include "scanner/joint_model/big_snake.h"

#include <cmath>
#include <cstdlib>
#include <optional>

#include "scanner/image/camera_model.h"
#include "scanner/image/image.h"
#include "scanner/image/image_builder.h"
#include "scanner/image/tilted_perspective_camera.h"
#include "scanner/joint_model/joint_model.h"
#include "scanner/scanner_configuration.h"
#include "common/logging/application_log.h"
#ifndef DOCTEST_CONFIG_DISABLE
#include <doctest/doctest.h>

// NOLINTBEGIN(*-magic-numbers, *-optional-access, *-use-nodiscard)

struct TestData {
  scanner::joint_model::JointProperties joint_properties{};
  scanner::ScannerConfigurationData scanner_config{};
  std::unique_ptr<scanner::image::TiltedPerspectiveCamera> camera_model;
  struct {
    std::string name;
    std::string path;
  } image_data;
};

inline auto Setup() -> TestData {
  TestData test_data;
  scanner::joint_model::JointProperties jp = {.upper_joint_width           = 25.0,
                                              .left_max_surface_angle      = 0.34906585,
                                              .right_max_surface_angle     = 0.34906585,
                                              .left_joint_angle            = 0.16,
                                              .right_joint_angle           = 0.140,
                                              .groove_depth                = 42.0,
                                              .upper_joint_width_tolerance = 7.0,
                                              .surface_angle_tolerance     = 10.0 * std::numbers::pi / 180.0,
                                              .groove_angle_tolerance      = 9.0 * std::numbers::pi / 180.0,
                                              .offset_distance             = 3.0};

  scanner::image::TiltedPerspectiveCameraProperties camera_properties;

  camera_properties.config_calib.intrinsic = {
      .projection_center_distance = 0.0,
      .focus_distance             = 4.707852952290943804,
      .principal_point =
          {
                            .x = 1.001100742322118764,
                            .y = 7.317642435771299914e-01,
                            },
      .pixel_pitch =
          {
                            .x = 2.74e-06,
                            .y = 2.74e-06,
                            },
      .rho = 3.141447305679321289,
      .tau = 1.221730476396030718,
      .d   = 6.193863034310445048e-01,
      .K1  = 2.545519889414866316e-02,
      .K2  = 4.181119910248848152e-03,
      .K3  = -6.696371931147962128e-03,
      .P1  = -3.320003802347088265e-03,
      .P2  = 3.050356537053298695e-03,
      .scaling_factors =
          {
                            .w  = 5.633439999999999975e-03,
                            .m  = 0.1,
                            .K1 = 0.1,
                            .K2 = 0.1,
                            .K3 = 0.1,
                            .P1 = 0.1,
                            .P2 = 0.1,
                            },
  };
  camera_properties.config_calib.extrinsic.rotation.row(1) << 1.065018256781005875e-02, 4.068680551182541349e-01,
      -9.134248514987763912e-01;
  camera_properties.config_calib.extrinsic.rotation.row(2) << 2.099075955949937164e-02, 9.131844018800093776e-01,
      4.070056955082629324e-01;

  camera_properties.config_calib.extrinsic.rotation.row(0) << 9.997229424317457536e-01, -2.350816639678374523e-02,
      1.185111080670121601e-03;
  camera_properties.config_calib.extrinsic.rotation.row(1) << 1.065018256781005875e-02, 4.068680551182541349e-01,
      -9.134248514987763912e-01;
  camera_properties.config_calib.extrinsic.rotation.row(2) << 2.099075955949937164e-02, 9.131844018800093776e-01,
      4.070056955082629324e-01;
  camera_properties.config_calib.extrinsic.translation.col(0) << -5.106240047893689099e-02, -2.791469469541549980e-02,
      3.925539620524008955e-01;

  camera_properties.config_fov = {.width = 3500, .offset_x = 312, .height = 2500, .offset_y = 0};

  auto camera_model = std::make_unique<scanner::image::TiltedPerspectiveCamera>(camera_properties);

  test_data = {
      .joint_properties = jp,
      .scanner_config   = {.gray_minimum_wall = 16},
      .camera_model     = std::move(camera_model),
      .image_data = {.name = "Image__2024-08-16__11-13-03.tiff", .path = "./src/scanner/joint_model/test/test_data/"}
  };

  return test_data;
}

TEST_SUITE("Test Big Snake") {
  TEST_CASE("Parse") {
    auto test_data = Setup();
    LOG_INFO("Single-image test setup complete: image={}{}", test_data.image_data.path, test_data.image_data.name);

    auto big_snake = scanner::joint_model::BigSnake(test_data.joint_properties, test_data.scanner_config,
                                                    std::move(test_data.camera_model));
    LOG_INFO("Constructed BigSnake for single-image test");

    auto grayscale_image = imread(test_data.image_data.path + test_data.image_data.name, cv::IMREAD_GRAYSCALE);
    LOG_INFO("Loaded grayscale image: {}{} (rows={}, cols={})", test_data.image_data.path, test_data.image_data.name,
             grayscale_image.rows, grayscale_image.cols);
    auto maybe_image     = scanner::image::ImageBuilder::From(grayscale_image, test_data.image_data.name, 0).Finalize();
    auto *image          = maybe_image.value().get();

    auto res = big_snake.Parse(*image, {}, {}, false, {});
    if (res.has_value()) {
      auto [profile, snake_lpcs, processing_time, num_walls] = res.value();
      LOG_INFO("Parsed single image: processing_time={}ms, num_walls={}", processing_time, num_walls);
    } else {
      LOG_INFO("Parse failed for single image");
    }
    CHECK(res.has_value());
  }
}
#endif
