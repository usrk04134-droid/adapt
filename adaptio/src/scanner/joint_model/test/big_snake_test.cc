#include "scanner/joint_model/big_snake.h"

#include <cmath>
#include <numbers>
#include <cstdlib>
#include <optional>
#include <filesystem>

#include "common/file/yaml.h"
#include "scanner/image/camera_model.h"
#include "scanner/image/image.h"
#include "scanner/image/image_builder.h"
#include "scanner/image/tilted_perspective_camera.h"
#include "scanner/joint_model/joint_model.h"
#include "scanner/scanner_configuration.h"
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
  using common::file::Yaml;
  TestData test_data;

  const auto base_dir = std::filesystem::path(__FILE__).parent_path() / "test_data";
  const auto cam_yaml  = (base_dir / "new_cam_params_silverscanner.yaml").string();
  const auto geo_yaml  = (base_dir / "new_u_joint.yaml").string();

  auto maybe_scanner_cfg = Yaml::FromFile(cam_yaml, "camera");
  REQUIRE(maybe_scanner_cfg.has_value());
  auto scanner_cfg = maybe_scanner_cfg.value()->AsUnorderedMap();

  auto camera_properties       = scanner::image::TiltedPerspectiveCameraProperties::FromUnorderedMap(scanner_cfg);
  camera_properties.config_fov = {.width = 3500, .offset_x = 312, .height = 2500, .offset_y = 0};

  auto maybe_joint_geo = Yaml::FromFile(geo_yaml, "joint");
  REQUIRE(maybe_joint_geo.has_value());
  auto joint_geo_map = maybe_joint_geo.value()->AsUnorderedMap();

  scanner::joint_model::JointProperties jp = {
      .upper_joint_width           = joint_geo_map.at("joint/upper_joint_width").Value<double>().value(),
      .left_max_surface_angle      = joint_geo_map.at("joint/left_max_surface_angle").Value<double>().value(),
      .right_max_surface_angle     = joint_geo_map.at("joint/right_max_surface_angle").Value<double>().value(),
      .left_joint_angle            = joint_geo_map.at("joint/left_joint_angle").Value<double>().value(),
      .right_joint_angle           = joint_geo_map.at("joint/right_joint_angle").Value<double>().value(),
      .groove_depth                = joint_geo_map.at("joint/groove_depth").Value<double>().value(),
      .upper_joint_width_tolerance = 7.0,
      .surface_angle_tolerance     = 10.0 * std::numbers::pi / 180.0,
      .groove_angle_tolerance      = 9.0 * std::numbers::pi / 180.0,
      .offset_distance             = 3.0,
  };

  auto camera_model = std::make_unique<scanner::image::TiltedPerspectiveCamera>(camera_properties);

  test_data = {.joint_properties = jp,
               .scanner_config   = {.gray_minimum_top = 48, .gray_minimum_wall = 16, .gray_minimum_bottom = 48},
               .camera_model     = std::move(camera_model),
               .image_data       = {.name = "Image__2024-08-16__11-13-03.tiff",
                                 .path = (base_dir.string() + "/")}};

  return test_data;
}

TEST_SUITE("Test Big Snake") {
  TEST_CASE("Parse") {
    auto test_data = Setup();

    auto big_snake = scanner::joint_model::BigSnake(test_data.joint_properties, test_data.scanner_config,
                                                    std::move(test_data.camera_model));

    auto grayscale_image = imread(test_data.image_data.path + test_data.image_data.name, cv::IMREAD_GRAYSCALE);
    auto maybe_image     = scanner::image::ImageBuilder::From(grayscale_image, test_data.image_data.name, 0).Finalize();
    auto *image          = maybe_image.value().get();

    auto res = big_snake.Parse(*image, {}, {}, false, {});
    REQUIRE(res.has_value());

    auto parsed = res.value();
    auto &profile = std::get<0>(parsed);

    // Expected ABW points for Image__2024-08-16__11-13-03.tiff from data_set.yaml (id: 3)
    const double expected_xs[7] = {0.0394735, 0.0459291, 0.0494897, 0.0530746, 0.0566018, 0.06003, 0.064345};
    const double expected_ys[7] = {0.0459356, 0.00591587, 0.00567378, 0.00476669, 0.00585357, 0.0061066, 0.0444451};
    for (int i = 0; i < 7; i++) {
      CHECK(std::abs(profile.points[i].x - expected_xs[i]) < 0.002);
      CHECK(std::abs(profile.points[i].y - expected_ys[i]) < 0.002);
    }
  }
}
#endif
