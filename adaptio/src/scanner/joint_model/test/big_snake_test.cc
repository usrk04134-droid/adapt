#include "scanner/joint_model/big_snake.h"

#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "common/file/yaml.h"
#include "common/logging/application_log.h"
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
  scanner::image::TiltedPerspectiveCameraProperties camera_properties;
  std::filesystem::path images_path;
};

// Function to read image filename to ID mapping from YAML
std::unordered_map<std::string, int> CreateFilenameToIdMap(std::filesystem::path const& yaml_file_path) {
  std::unordered_map<std::string, int> filename_to_id_map;

  try {
    LOG_INFO("path is {}", yaml_file_path.string());
    YAML::Node config        = YAML::LoadFile(yaml_file_path.string());
    const YAML::Node& images = config["images"];

    for (const auto& image_node : images) {
      int image_id          = image_node["id"].as<int>();
      std::string file_path = image_node["file_path"].as<std::string>();

      // Extract just the filename from the path
      std::filesystem::path fs_path(file_path);
      std::string filename = fs_path.filename().string();

      filename_to_id_map[filename] = image_id;
    }
  } catch (const YAML::Exception& e) {
    LOG_ERROR("Failed to read YAML file for filename mapping: {}", e.what());
  } catch (const std::exception& e) {
    LOG_ERROR("Error creating filename to ID map: {}", e.what());
  }

  return filename_to_id_map;
}

// Function to read annotations from YAML file
bool ReadAnnotationsFromYAML(std::filesystem::path const& file_path, int image_id, std::vector<double>& xs,
                             std::vector<double>& ys) {
  try {
    YAML::Node config             = YAML::LoadFile(file_path.string());
    const YAML::Node& annotations = config["annotations"];
    for (const auto& annotation : annotations) {
      int current_image_id = annotation["image_id"].as<int>();
      if (current_image_id == image_id) {
        xs = annotation["abw_points_xs"].as<std::vector<double>>();
        ys = annotation["abw_points_ys"].as<std::vector<double>>();
        return true;
      }
    }
  } catch (const YAML::Exception& e) {
    LOG_ERROR("Failed to read YAML file: {}", e.what());
  }
  return false;
}

// Function to get all image files from directory
std::vector<std::filesystem::path> GetImageFiles(std::filesystem::path const& directory_path) {
  std::vector<std::filesystem::path> image_files;

  try {
    for (const auto& entry : std::filesystem::directory_iterator(directory_path)) {
      if (entry.is_regular_file()) {
        std::string extension = entry.path().extension().string();
        std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char c) { return std::tolower(c); });

        if (extension == ".tiff") {
          image_files.push_back(entry.path());
        }
      }
    }
  } catch (const std::filesystem::filesystem_error& e) {
    LOG_ERROR("Failed to read directory {}: {}", directory_path.string(), e.what());
  }

  return image_files;
}

inline auto Setup() -> TestData {
  TestData test_data;
  auto base_dir = std::filesystem::path(__FILE__).parent_path() / "test_data";

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

  camera_properties.config_calib.extrinsic.rotation.row(0) << 9.997229424317457536e-01, -2.350816639678374523e-02,
      1.185111080670121601e-03;
  camera_properties.config_calib.extrinsic.rotation.row(1) << 1.065018256781005875e-02, 4.068680551182541349e-01,
      -9.134248514987763912e-01;
  camera_properties.config_calib.extrinsic.rotation.row(2) << 2.099075955949937164e-02, 9.131844018800093776e-01,
      4.070056955082629324e-01;
  camera_properties.config_calib.extrinsic.translation.col(0) << -5.106240047893689099e-02, -2.791469469541549980e-02,
      3.925539620524008955e-01;

  camera_properties.config_fov = {.width = 3500, .offset_x = 312, .height = 2500, .offset_y = 0};

  test_data = {.joint_properties  = jp,
               .scanner_config    = {.gray_minimum_wall = 24},
               .camera_properties = camera_properties,
               .images_path       = base_dir / "images"};

  return test_data;
}

TEST_SUITE("Test Big Snake") {
  TEST_CASE("Parse All Images") {
    auto test_data = Setup();
    auto base_dir  = std::filesystem::path(__FILE__).parent_path() / "test_data";
    std::filesystem::path yaml_file_path = base_dir / "data_set.yaml";

    // Create filename to image ID mapping dynamically from YAML
    auto filename_to_id_map = CreateFilenameToIdMap(yaml_file_path);
    REQUIRE(!filename_to_id_map.empty());

    // Get all image files from the directory
    auto image_files = GetImageFiles(test_data.images_path);
    REQUIRE(!image_files.empty());

    for (const auto& image_path : image_files) {
      SUBCASE(("Processing: " + image_path.filename().string()).c_str()) {
        // Create a new camera model for each test case using the properties
        auto camera_model = std::make_unique<scanner::image::TiltedPerspectiveCamera>(test_data.camera_properties);

        auto big_snake = scanner::joint_model::BigSnake(test_data.joint_properties, test_data.scanner_config,
                                                        std::move(camera_model));

        auto grayscale_image = cv::imread(image_path.string(), cv::IMREAD_GRAYSCALE);
        REQUIRE(!grayscale_image.empty());

        auto maybe_image =
            scanner::image::ImageBuilder::From(grayscale_image, image_path.filename().string(), 0).Finalize();
        auto* image = maybe_image.value().get();

        // Get image ID from the dynamic mapping
        std::string filename = image_path.filename().string();
        auto it              = filename_to_id_map.find(filename);

        if (it == filename_to_id_map.end()) {
          LOG_INFO("Skipping {} - no image ID mapping found in YAML", filename);
          continue;
        }

        int image_id = it->second;

        // Read annotations from YAML file for this image ID
        std::vector<double> expected_xs, expected_ys;
        if (!ReadAnnotationsFromYAML(yaml_file_path, image_id, expected_xs, expected_ys)) {
          LOG_INFO("Failed to read annotations for image ID {} ({})", image_id, filename);
          continue;
        }

        auto res = big_snake.Parse(*image, {}, {}, false, {});
        if (!res.has_value()) {
          LOG_INFO("Failed to parse image: {}", filename);
          continue;
        }

        auto parsed   = res.value();
        auto& profile = std::get<0>(parsed);

        LOG_INFO("Testing image: {}", filename);
        for (int i = 0; i < 7; i++) {
          LOG_INFO("  Point {}: parsed ({}, {}) vs expected ({}, {})", i, profile.points[i].x, profile.points[i].y,
                   expected_xs[i], expected_ys[i]);
          CHECK(std::abs(profile.points[i].x - expected_xs[i]) < 0.49);
          CHECK(std::abs(profile.points[i].y - expected_ys[i]) < 0.49);
        }
      }
    }
  }
}
#endif
