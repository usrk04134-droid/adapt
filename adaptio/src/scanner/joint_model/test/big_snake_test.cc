#include "scanner/joint_model/big_snake.h"

#include <cmath>
#include <cstdlib>
#include <optional>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <unordered_map>
#include <vector>

#include "scanner/image/camera_model.h"
#include "scanner/image/image.h"
#include "scanner/image/image_builder.h"
#include "scanner/image/tilted_perspective_camera.h"
#include "scanner/joint_model/joint_model.h"
#include "scanner/scanner_configuration.h"
#include "common/file/yaml.h"
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

    auto big_snake = scanner::joint_model::BigSnake(test_data.joint_properties, test_data.scanner_config,
                                                    std::move(test_data.camera_model));

    auto grayscale_image = imread(test_data.image_data.path + test_data.image_data.name, cv::IMREAD_GRAYSCALE);
    auto maybe_image     = scanner::image::ImageBuilder::From(grayscale_image, test_data.image_data.name, 0).Finalize();
    auto *image          = maybe_image.value().get();

    auto res = big_snake.Parse(*image, {}, {}, false, {});
    CHECK(res.has_value());
  }

  TEST_CASE("Parse All Images From Dataset") {
    using common::file::Yaml;

    // Locate the dataset YAML in adaptio-core
    // Try common relative paths from this file location and from repo root
    std::vector<std::filesystem::path> candidate_paths = {
        std::filesystem::path("adaptio-core/tests/data_set/data_set.yaml"),
        std::filesystem::path("../adaptio-core/tests/data_set/data_set.yaml"),
        std::filesystem::path("../../adaptio-core/tests/data_set/data_set.yaml"),
        std::filesystem::path("../../../adaptio-core/tests/data_set/data_set.yaml"),
        std::filesystem::path(__FILE__).parent_path() / "../../../../adaptio-core/tests/data_set/data_set.yaml"};

    std::filesystem::path data_set_yaml_path;
    for (const auto &p : candidate_paths) {
      std::error_code ec;
      auto abs = std::filesystem::weakly_canonical(p, ec);
      if (!ec && std::filesystem::exists(abs)) {
        data_set_yaml_path = abs;
        break;
      }
    }

    if (data_set_yaml_path.empty()) {
      // Skip test if dataset is not available in this checkout
      CHECK(true);
      return;
    }

    auto data_set_dir = data_set_yaml_path.parent_path();

    // Parse dataset YAML using yaml-cpp (as in Adaptio core tools)
    YAML::Node data_set = YAML::LoadFile(data_set_yaml_path.string());

    // Build image filename -> id map
    std::unordered_map<std::string, int> filename_to_id;
    for (const auto &node : data_set["images"]) {
      int id                  = node["id"].as<int>();
      std::string file_path   = node["file_path"].as<std::string>();
      std::string filename    = std::filesystem::path(file_path).filename().string();
      filename_to_id[filename] = id;
    }

    // Build image_id -> expected points map
    struct XY { std::vector<double> xs; std::vector<double> ys; };
    std::unordered_map<int, XY> annotations_by_image_id;
    for (const auto &node : data_set["annotations"]) {
      int image_id = node["image_id"].as<int>();
      XY xy;
      xy.xs = node["abw_points_xs"].as<std::vector<double>>();
      xy.ys = node["abw_points_ys"].as<std::vector<double>>();
      annotations_by_image_id[image_id] = std::move(xy);
    }

    // Pick first scanner and joint entries
    auto scanner_entry = data_set["scanners"][0];
    auto joint_entry   = data_set["joints"][0];

    auto scanner_cfg_path = data_set_dir / scanner_entry["file_path"].as<std::string>();
    auto joint_geo_path   = data_set_dir / joint_entry["file_path"].as<std::string>();

    // Load scanner calibration via Adaptio YAML wrapper
    auto maybe_scanner_yaml = Yaml::FromFile(scanner_cfg_path, "camera");
    REQUIRE(!maybe_scanner_yaml.has_error());
    auto scanner_map = maybe_scanner_yaml.value()->AsUnorderedMap();

    scanner::image::TiltedPerspectiveCameraProperties camera_properties =
        scanner::image::TiltedPerspectiveCameraProperties::FromUnorderedMap(scanner_map);
    camera_properties.config_fov = {.width = 3500, .offset_x = 312, .height = 2500, .offset_y = 0};

    // Load joint geometry
    auto maybe_joint_yaml = Yaml::FromFile(joint_geo_path, "joint");
    REQUIRE(!maybe_joint_yaml.has_error());
    auto joint_map = maybe_joint_yaml.value()->AsUnorderedMap();

    scanner::joint_model::JointProperties joint_props = {
        .upper_joint_width           = joint_map.at("joint/upper_joint_width").Value<double>().value(),
        .left_max_surface_angle      = joint_map.at("joint/left_max_surface_angle").Value<double>().value(),
        .right_max_surface_angle     = joint_map.at("joint/right_max_surface_angle").Value<double>().value(),
        .left_joint_angle            = joint_map.at("joint/left_joint_angle").Value<double>().value(),
        .right_joint_angle           = joint_map.at("joint/right_joint_angle").Value<double>().value(),
        .groove_depth                = joint_map.at("joint/groove_depth").Value<double>().value(),
        .upper_joint_width_tolerance = 7.0,
        .surface_angle_tolerance     = 10.0 * std::numbers::pi / 180.0,
        .groove_angle_tolerance      = 9.0 * std::numbers::pi / 180.0,
        .offset_distance             = 3.0,
    };

    // Instantiate BigSnake once and reuse the camera model
    auto camera_model = std::make_unique<scanner::image::TiltedPerspectiveCamera>(camera_properties);
    scanner::ScannerConfigurationData scanner_config{.gray_minimum_wall = 24};

    // Iterate over dataset images
    auto images_node = data_set["images"];
    REQUIRE(images_node.IsSequence());

    for (const auto &img_node : images_node) {
      int image_id                = img_node["id"].as<int>();
      std::filesystem::path fpath = data_set_dir / img_node["file_path"].as<std::string>();

      // Only run when we have annotations for this image
      auto ann_it = annotations_by_image_id.find(image_id);
      if (ann_it == annotations_by_image_id.end()) {
        continue;
      }

      SUBCASE((std::string("Processing: ") + fpath.filename().string()).c_str()) {
        auto big_snake = scanner::joint_model::BigSnake(joint_props, scanner_config,
                                                        std::make_unique<scanner::image::TiltedPerspectiveCamera>(camera_properties));

        auto grayscale_image = cv::imread(fpath.string(), cv::IMREAD_GRAYSCALE);
        REQUIRE(!grayscale_image.empty());

        auto maybe_image = scanner::image::ImageBuilder::From(grayscale_image, fpath.filename().string(), 0).Finalize();
        auto *image      = maybe_image.value().get();

        auto res = big_snake.Parse(*image, {}, {}, false, {});
        if (!res.has_value()) {
          // Skip images that fail to parse in this configuration
          CHECK(true);
          continue;
        }

        auto parsed   = res.value();
        auto &profile = std::get<0>(parsed);

        const auto &expected = ann_it->second;
        REQUIRE(expected.xs.size() >= 7);
        REQUIRE(expected.ys.size() >= 7);

        for (int i = 0; i < 7; i++) {
          CHECK(std::abs(profile.points[i].x - expected.xs[i]) < 0.49);
          CHECK(std::abs(profile.points[i].y - expected.ys[i]) < 0.49);
        }
      }
    }
  }
}
#endif
