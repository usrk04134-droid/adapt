#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
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

auto ListImageFiles(const std::filesystem::path &images_dir) -> std::vector<std::filesystem::path> {
  std::vector<std::filesystem::path> files;
  if (!std::filesystem::exists(images_dir)) {
    return files;
  }
  for (const auto &entry : std::filesystem::directory_iterator(images_dir)) {
    if (entry.is_regular_file()) {
      auto ext = entry.path().extension().string();
      std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
      if (ext == ".tiff") {
        files.push_back(entry.path());
      }
    }
  }
  return files;
}

}  // namespace

TEST_SUITE("BigSnake dataset") {
  TEST_CASE("Parse matches annotated ABW points within tolerance") {
    // Arrange
    const std::filesystem::path base_dir   = std::filesystem::path(__FILE__).parent_path() / "test_data";
    const std::string dataset_path         = (base_dir / "data_set.yaml").string();
    const auto dataset             = LoadDataset(dataset_path);

    REQUIRE(dataset.images.size() >= 1);

    // Resolve scanner/joint paths relative to dataset yaml directory
    const std::filesystem::path dataset_dir = std::filesystem::path(dataset_path).parent_path();
    const auto scanner_cfg_path             = (dataset_dir / dataset.scanners.at(2).file_path).string();
    const auto joint_geo_path               = (dataset_dir / dataset.joints.at(2).file_path).string();

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

    // Build filename -> image entry map
    std::unordered_map<std::string, DatasetImage> filename_to_image;
    for (const auto &img : dataset.images) {
      filename_to_image[std::filesystem::path(img.file_path).filename().string()] = img;
    }

    // Iterate images present in directory; skip those without mapping/annotations
    int tested = 0;
    const auto images_dir = dataset_dir / "images";
    for (const auto &image_path : ListImageFiles(images_dir)) {
      const std::string filename = image_path.filename().string();
      auto it_img                = filename_to_image.find(filename);
      if (it_img == filename_to_image.end()) {
        continue;
      }
      const auto &img_entry = it_img->second;
      const auto ann_it     = dataset.annotations_by_image_id.find(img_entry.id);
      if (ann_it == dataset.annotations_by_image_id.end()) {
        continue;
      }
      const auto &annotation = ann_it->second;

      auto gray = imread(image_path.string(), cv::IMREAD_GRAYSCALE);
      if (gray.data == nullptr) {
        continue;
      }
      auto maybe_image = scanner::image::ImageBuilder::From(gray, filename, 0).Finalize();
      REQUIRE(maybe_image.has_value());
      auto *image = maybe_image.value().get();

      auto res = snake.Parse(*image, {}, {}, false, {});
      REQUIRE(res.has_value());
      auto [profile, /*snake_lpcs*/, /*processing_time*/, /*num_walls*/] = res.value();

      REQUIRE(profile.points.size() == 7);
      REQUIRE(annotation.xs.size() == 7);
      REQUIRE(annotation.ys.size() == 7);

      const double tol_mm = 1.5;  // tolerance in mm
      for (size_t i = 0; i < 7; ++i) {
        const double dx_mm = (profile.points[i].x - annotation.xs[i]) * 1000.0;
        const double dy_mm = (profile.points[i].y - annotation.ys[i]) * 1000.0;
        const double dist  = std::sqrt(dx_mm * dx_mm + dy_mm * dy_mm);
        CHECK_MESSAGE(dist <= tol_mm, "image_id=" << img_entry.id << ": ABW" << i << " distance " << dist << "mm exceeds tolerance");
      }
      tested++;
    }
    CHECK(tested >= 1);
  }
}

#endif

