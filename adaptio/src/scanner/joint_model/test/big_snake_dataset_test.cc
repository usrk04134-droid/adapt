#include <yaml-cpp/yaml.h>

#include <cmath>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "scanner/image/image.h"
#include "scanner/image/image_builder.h"
#include "scanner/image/tilted_perspective_camera.h"
#include "scanner/joint_model/big_snake.h"
#include "scanner/joint_model/joint_model.h"
#include "scanner/scanner_configuration.h"
#include "common/file/yaml.h"

#ifndef DOCTEST_CONFIG_DISABLE
#include <doctest/doctest.h>

namespace {
struct DatasetImage {
  int id;
  std::string file_path;
  int joint_id;
  int scanner_id;
};

struct DatasetScanner {
  std::string file_path;
};

struct DatasetJoint {
  std::string file_path;
};

struct DatasetAnnotation {
  int image_id;
  std::vector<double> xs;
  std::vector<double> ys;
};

struct Dataset {
  std::vector<DatasetImage> images;
  std::unordered_map<int, DatasetScanner> scanners;
  std::unordered_map<int, DatasetJoint> joints;
  std::unordered_map<int, DatasetAnnotation> annotations_by_image_id;
};

auto LoadDataset(const std::string &dataset_yaml_path) -> Dataset {
  YAML::Node ds = YAML::LoadFile(dataset_yaml_path);

  Dataset dataset;
  for (auto it = ds["images"].begin(); it != ds["images"].end(); ++it) {
    dataset.images.push_back({(*it)["id"].as<int>(), (*it)["file_path"].as<std::string>(),
                              (*it)["joint_id"].as<int>(), (*it)["scanner_id"].as<int>()});
  }

  for (auto it = ds["scanners"].begin(); it != ds["scanners"].end(); ++it) {
    int id                                      = (*it)["id"].as<int>();
    dataset.scanners[id]                         = {(*it)["file_path"].as<std::string>()};
  }

  for (auto it = ds["joints"].begin(); it != ds["joints"].end(); ++it) {
    int id                                   = (*it)["id"].as<int>();
    dataset.joints[id]                        = {(*it)["file_path"].as<std::string>()};
  }

  for (auto it = ds["annotations"].begin(); it != ds["annotations"].end(); ++it) {
    int image_id                                      = (*it)["image_id"].as<int>();
    dataset.annotations_by_image_id[image_id]         = {image_id,
                                                         (*it)["abw_points_xs"].as<std::vector<double>>(),
                                                         (*it)["abw_points_ys"].as<std::vector<double>>()} 
    ;
  }

  return dataset;
}

}  // namespace

TEST_SUITE("BigSnake dataset") {
  TEST_CASE("Parse matches annotated ABW points within tolerance") {
    // Arrange
    const std::string dataset_path = "./src/scanner/joint_model/test/test_data/abw_dataset.yaml";
    const auto dataset             = LoadDataset(dataset_path);

    // Use first/only image entry
    REQUIRE(dataset.images.size() >= 1);
    const auto img_entry = dataset.images.front();
    const auto ann_it    = dataset.annotations_by_image_id.find(img_entry.id);
    REQUIRE(ann_it != dataset.annotations_by_image_id.end());
    const auto &annotation = ann_it->second;

    const auto scanner_cfg_path = dataset.scanners.at(img_entry.scanner_id).file_path;
    const auto joint_geo_path   = dataset.joints.at(img_entry.joint_id).file_path;

    // Load scanner and joint yaml using existing YAML utility
    auto maybe_scanner_yaml = common::file::Yaml::FromFile(scanner_cfg_path, "camera");
    REQUIRE_FALSE(maybe_scanner_yaml.has_error());
    auto scanner_cfg_map = maybe_scanner_yaml.value()->AsUnorderedMap();

    auto maybe_joint_yaml = common::file::Yaml::FromFile(joint_geo_path, "joint");
    REQUIRE_FALSE(maybe_joint_yaml.has_error());
    auto joint_map = maybe_joint_yaml.value()->AsUnorderedMap();

    // Build camera
    auto camera_properties       = scanner::image::TiltedPerspectiveCameraProperties::FromUnorderedMap(scanner_cfg_map);
    camera_properties.config_fov = {.width = 3500, .offset_x = 312, .height = 2500, .offset_y = 0};
    auto camera_model            = std::make_unique<scanner::image::TiltedPerspectiveCamera>(camera_properties);

    // Build BigSnake
    const scanner::joint_model::JointProperties properties = {
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

    scanner::ScannerConfigurationData scan_cfg{.gray_minimum_top = 48, .gray_minimum_wall = 24, .gray_minimum_bottom = 48};

    auto snake = scanner::joint_model::BigSnake(properties, scan_cfg, std::move(camera_model));

    // Load image
    const std::string image_path = std::string("./src/scanner/joint_model/test/test_data/") + img_entry.file_path;
    auto gray                    = imread(image_path, cv::IMREAD_GRAYSCALE);
    REQUIRE(gray.data != nullptr);
    auto maybe_image = scanner::image::ImageBuilder::From(gray, img_entry.file_path, 0).Finalize();
    REQUIRE(maybe_image.has_value());
    auto *image = maybe_image.value().get();

    // Act
    auto res = snake.Parse(*image, {}, {}, false, {});
    REQUIRE(res.has_value());
    auto [profile, /*snake_lpcs*/, /*processing_time*/, /*num_walls*/] = res.value();

    // Assert: ABW points close to annotated values
    REQUIRE(profile.points.size() == 7);
    REQUIRE(annotation.xs.size() == 7);
    REQUIRE(annotation.ys.size() == 7);

    const double tol_mm = 1.5;  // tolerance in mm
    for (size_t i = 0; i < 7; ++i) {
      const double dx_mm = (profile.points[i].x - annotation.xs[i]) * 1000.0;
      const double dy_mm = (profile.points[i].y - annotation.ys[i]) * 1000.0;
      const double dist  = std::sqrt(dx_mm * dx_mm + dy_mm * dy_mm);
      CHECK_MESSAGE(dist <= tol_mm, "ABW" << i << " distance " << dist << "mm exceeds tolerance");
    }
  }
}

#endif

