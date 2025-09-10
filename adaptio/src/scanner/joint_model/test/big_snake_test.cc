#include "scanner/joint_model/big_snake.h"

#include <cmath>
#include <cstdlib>
#include <optional>
#include <filesystem>
#include <map>
#include <vector>
#include <numbers>
#include <sstream>
#include <algorithm>
#include <yaml-cpp/yaml.h>

#include "scanner/image/camera_model.h"
#include "scanner/image/image.h"
#include "scanner/image/image_builder.h"
#include "scanner/image/tilted_perspective_camera.h"
#include "common/file/yaml.h"
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

    auto grayscale_image = cv::imread(test_data.image_data.path + test_data.image_data.name, cv::IMREAD_GRAYSCALE);
    auto maybe_image     = scanner::image::ImageBuilder::From(grayscale_image, test_data.image_data.name, 0).Finalize();
    auto *image          = maybe_image.value().get();

    auto res = big_snake.Parse(*image, {}, {}, false, {});
    CHECK(res.has_value());
  }

  TEST_CASE("Parse dataset images and compare ABW points to annotations") {
    // Try to locate the dataset directory. Prefer a copy under adaptio if present,
    // otherwise fall back to the legacy path under adaptio-core within this workspace.
    const auto here = std::filesystem::path(__FILE__).parent_path();
    std::vector<std::filesystem::path> candidate_base_dirs = {
        here / "data_set",
        here / "../../../../../adaptio-core/tests/data_set",
    };

    std::filesystem::path base_dir;
    for (const auto& cand : candidate_base_dirs) {
      if (std::filesystem::exists(cand / "data_set.yaml")) {
        base_dir = cand;
        break;
      }
    }

    REQUIRE_MESSAGE(!base_dir.empty(), "Could not locate data_set.yaml for dataset-driven tests");

    auto dataset = YAML::LoadFile((base_dir / "data_set.yaml").string());

    // Build image index
    struct ImageDef {
      int id;
      std::string path;
      int joint_id;
      int scanner_id;
    };
    std::vector<ImageDef> images;
    for (auto it = dataset["images"].begin(); it != dataset["images"].end(); ++it) {
      images.push_back({(*it)["id"].as<int>(), (*it)["file_path"].as<std::string>(), (*it)["joint_id"].as<int>(),
                        (*it)["scanner_id"].as<int>()});
    }

    // Build scanner index
    struct ScannerDef { std::string path; };
    std::map<int, ScannerDef> scanners;
    for (auto it = dataset["scanners"].begin(); it != dataset["scanners"].end(); ++it) {
      scanners.insert({(*it)["id"].as<int>(), ScannerDef{(*it)["file_path"].as<std::string>()}});
    }

    // Build joint index
    struct JointDef { std::string path; };
    std::map<int, JointDef> joints;
    for (auto it = dataset["joints"].begin(); it != dataset["joints"].end(); ++it) {
      joints.insert({(*it)["id"].as<int>(), JointDef{(*it)["file_path"].as<std::string>()}});
    }

    // Build annotations index by image_id
    struct Annotation { std::vector<double> xs; std::vector<double> ys; };
    std::map<int, Annotation> annotations;
    for (auto it = dataset["annotations"].begin(); it != dataset["annotations"].end(); ++it) {
      const int image_id = (*it)["image_id"].as<int>();
      Annotation a{(*it)["abw_points_xs"].as<std::vector<double>>(),
                   (*it)["abw_points_ys"].as<std::vector<double>>()};
      annotations.insert({image_id, std::move(a)});
    }

    // Tolerances in mm
    const double max_point_distance_mm = 1.5;

    int tested = 0;
    for (const auto& img : images) {
      // Skip images without annotations
      if (!annotations.contains(img.id)) {
        continue;
      }

      // Load camera configuration
      auto scanner_cfg_path = base_dir / scanners.at(img.scanner_id).path;
      auto maybe_scanner_cfg = common::file::Yaml::FromFile(scanner_cfg_path, "camera");
      REQUIRE_MESSAGE(maybe_scanner_cfg.has_value(), "Failed to load scanner config: " + scanner_cfg_path.string());
      auto scanner_cfg_map = maybe_scanner_cfg.value()->AsUnorderedMap();

      // Load joint geometry
      auto joint_geo_path = base_dir / joints.at(img.joint_id).path;
      auto maybe_joint_geo = common::file::Yaml::FromFile(joint_geo_path, "joint");
      REQUIRE_MESSAGE(maybe_joint_geo.has_value(), "Failed to load joint geometry: " + joint_geo_path.string());
      auto joint_geo_map = maybe_joint_geo.value()->AsUnorderedMap();

      // Camera model
      scanner::image::TiltedPerspectiveCameraProperties camera_properties;
      camera_properties = scanner::image::TiltedPerspectiveCameraProperties::FromUnorderedMap(scanner_cfg_map);
      // FOV used by the legacy test setup
      camera_properties.config_fov = {.width = 3500, .offset_x = 312, .height = 2500, .offset_y = 0};
      auto camera_model = std::make_unique<scanner::image::TiltedPerspectiveCamera>(camera_properties);

      // Joint properties
      const auto properties = scanner::joint_model::JointProperties{
          .upper_joint_width = joint_geo_map.at("joint/upper_joint_width").Value<double>().value(),
          .left_max_surface_angle = joint_geo_map.at("joint/left_max_surface_angle").Value<double>().value(),
          .right_max_surface_angle = joint_geo_map.at("joint/right_max_surface_angle").Value<double>().value(),
          .left_joint_angle = joint_geo_map.at("joint/left_joint_angle").Value<double>().value(),
          .right_joint_angle = joint_geo_map.at("joint/right_joint_angle").Value<double>().value(),
          .groove_depth = joint_geo_map.at("joint/groove_depth").Value<double>().value(),
          .upper_joint_width_tolerance = 7.0,
          .surface_angle_tolerance = 10.0 * std::numbers::pi / 180.0,
          .groove_angle_tolerance = 9.0 * std::numbers::pi / 180.0,
          .offset_distance = 3.0,
      };

      // Scanner gray thresholds (allow per-image override via scanner YAML if present)
      int gray_top = 48;
      int gray_wall = 16;
      int gray_bottom = 48;
      if (scanner_cfg_map.contains("camera/image_processing/gray_minimum_top")) {
        gray_top = static_cast<int>(scanner_cfg_map.at("camera/image_processing/gray_minimum_top").Value<double>().value());
      }
      if (scanner_cfg_map.contains("camera/image_processing/gray_minimum_wall")) {
        gray_wall = static_cast<int>(scanner_cfg_map.at("camera/image_processing/gray_minimum_wall").Value<double>().value());
      }
      if (scanner_cfg_map.contains("camera/image_processing/gray_minimum_bottom")) {
        gray_bottom = static_cast<int>(scanner_cfg_map.at("camera/image_processing/gray_minimum_bottom").Value<double>().value());
      }

      scanner::joint_model::BigSnake big_snake(properties, {.gray_minimum_top = gray_top, .gray_minimum_wall = gray_wall, .gray_minimum_bottom = gray_bottom},
                                               std::move(camera_model));

      auto image_path = base_dir / img.path;
      auto grayscale_image = cv::imread(image_path.string(), cv::IMREAD_GRAYSCALE);
      REQUIRE_MESSAGE(!grayscale_image.empty(), "Failed to read image: " + image_path.string());
      auto maybe_image = scanner::image::ImageBuilder::From(grayscale_image, image_path.filename().string(), 0).Finalize();
      REQUIRE(maybe_image.has_value());
      auto* image = maybe_image.value().get();

      auto res = big_snake.Parse(*image, {}, {}, false, {});
      REQUIRE_MESSAGE(res.has_value(), "BigSnake::Parse failed for image: " + image_path.string());

      const auto& profile = std::get<0>(res.value());
      const auto& expected = annotations.at(img.id);
      REQUIRE(expected.xs.size() == 7);
      REQUIRE(expected.ys.size() == 7);

      double max_dist_mm = 0.0;
      for (int i = 0; i < 7; i++) {
        const double dx_mm = expected.xs[i] * 1000.0 - profile.points[i].x * 1000.0;
        const double dy_mm = expected.ys[i] * 1000.0 - profile.points[i].y * 1000.0;
        const double dist_mm = std::sqrt(dx_mm * dx_mm + dy_mm * dy_mm);
        max_dist_mm = std::max(max_dist_mm, dist_mm);
      }

      CHECK_MESSAGE(max_dist_mm <= max_point_distance_mm,
                    (std::stringstream() << "Max ABW deviation " << max_dist_mm
                                          << " mm exceeds limit for image id " << img.id << " ("
                                          << image_path.filename().string() << ")")
                        .str());
      tested++;
    }

    CHECK_MESSAGE(tested > 0, "No dataset images with annotations were tested");
  }
}
#endif
