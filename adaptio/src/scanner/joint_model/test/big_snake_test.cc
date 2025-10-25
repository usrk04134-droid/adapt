#include "scanner/joint_model/big_snake.h"

#include <fmt/std.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <unordered_map>
#include <vector>

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

struct AbwPoint {
  double x;
  double y;
};

struct ImageTestData {
  std::string filename;
  int image_id;
  std::vector<AbwPoint> expected_points;
};

struct TestData {
  scanner::joint_model::JointProperties joint_properties{};
  scanner::ScannerConfigurationData scanner_config{};
  scanner::image::TiltedPerspectiveCameraProperties camera_properties;
  std::filesystem::path images_path;
  std::vector<ImageTestData> image_test_data;
};

// Fetch only files listed in image_test_data (and ending with .tiff)
static std::vector<std::filesystem::path> GetImageFiles(std::filesystem::path const& images_path,
                                                        const std::vector<ImageTestData>& listed) {
  std::vector<std::filesystem::path> image_files;
  image_files.reserve(listed.size());

  for (const auto& td : listed) {
    std::filesystem::path p = images_path / td.filename;

    // Normalize path and check extension
    std::filesystem::path abs = std::filesystem::weakly_canonical(p);
    std::string ext           = abs.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    if (ext == ".tiff") {
      if (std::filesystem::exists(abs)) {
        image_files.push_back(abs);
      } else {
        LOG_INFO("Listed test image not found, skipping: {} (looked at {})", td.filename, abs);
      }
    } else {
      LOG_INFO("Skipping non-tiff listed file: {}", td.filename);
    }
  }
  return image_files;
}

inline auto Setup() -> std::vector<TestData> {
  std::vector<TestData> test_data_vector;

  // Create a single TestData object (you can modify this to create multiple TestData objects if needed)
  TestData test_data;
  std::filesystem::path const base_dir = std::filesystem::path(__FILE__).parent_path() / "test_data";

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
      .focus_distance             = 4.707852952290943804e+00,
      .principal_point =
          {
                            .x = 1.001100742322118764e+00,
                            .y = 7.317642435771299914e-01,
                            },
      .pixel_pitch =
          {
                            .x = 2.74e-06,
                            .y = 2.74e-06,
                            },

      .rho = 3.141447305679321289e+00,
      .tau = 1.221730476396030718e-01,
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

  // Hardcoded annotation data from the YAML file
  std::vector<ImageTestData> image_test_data = {
      {"Image__2024-08-16__11-13-03.tiff",
       3,  {{0.0394735, 0.0459356},
        {0.0459291, 0.00591587},
        {0.0494897, 0.00567378},
        {0.0530746, 0.00476669},
        {0.0566018, 0.00585357},
        {0.06003, 0.0061066},
        {0.064345, 0.0444451}}   },
      {"Image__2024-08-16__11-13-22.tiff",
       4,  {{0.0394383, 0.0461375},
        {0.0459291, 0.00591587},
        {0.0494864, 0.00583949},
        {0.0530722, 0.00484993},
        {0.0566356, 0.00602021},
        {0.0601783, 0.0063588},
        {0.0644133, 0.0445152}}  },
      {"Image__2024-08-16__11-15-08.tiff",
       5,  {{0.040356, 0.0355344},
        {0.0470767, -0.00475701},
        {0.050607, -0.00501648},
        {0.054115, -0.00571419},
        {0.0576089, -0.00466403},
        {0.0610746, -0.00422705},
        {0.0653201, 0.0340763}}  },
      {"Image__2024-08-16__11-15-17.tiff",
       6,  {{0.0405664, 0.0265532},
        {0.0469914, -0.0136532},
        {0.0505119, -0.0139272},
        {0.054054, -0.0147483},
        {0.0575487, -0.0139296},
        {0.0610521, -0.013567},
        {0.0654341, 0.0249215}}  },
      {"Image__2024-08-16__11-15-26.tiff",
       7,  {{0.0401842, 0.0412982},
        {0.0467125, 0.00107684},
        {0.0502326, 0.000911142},
        {0.05382, -1.8682e-05},
        {0.0573522, 0.000921895},
        {0.0608533, 0.00135055},
        {0.065084, 0.0396181}}   },
      {"Image__2024-08-16__11-15-32.tiff",
       8,  {{0.0357308, 0.0414683},
        {0.0420649, 0.00138364},
        {0.0456207, 0.00113413},
        {0.049193, 0.000290129},
        {0.0527338, 0.00114401},
        {0.0562766, 0.00157272},
        {0.0605858, 0.0398535}}  },
      {"Image__2024-08-16__11-15-40.tiff",
       9,  {{0.0422294, 0.0414862},
        {0.0486052, 0.00146319},
        {0.0521682, 0.00112969},
        {0.0557557, 0.000286076},
        {0.0592449, 0.00113988},
        {0.0627834, 0.0015694},
        {0.0670895, 0.0398799}}  },
      {"top_surface_mismatch.tiff",
       10, {{0.0450931, -0.0635028},
        {0.0514282, -0.103693},
        {0.0550256, -0.103995},
        {0.0585907, -0.104698},
        {0.0621532, -0.103806},
        {0.0657357, -0.103579},
        {0.0709831, -0.0579145}}},
      {"Image__2024-08-20__11-34-43.tiff",
       11, {{0.0383158, -0.0509335},
        {0.0448204, -0.0910697},
        {0.0483731, -0.0912273},
        {0.051942, -0.0920177},
        {0.0554734, -0.0910389},
        {0.0590142, -0.0906932},
        {0.0634224, -0.0523225}}},
      {"Image__2024-08-20__11-34-49.tiff",
       12, {{0.0350469, -0.0509117},
        {0.0414678, -0.0910334},
        {0.0450194, -0.0913168},
        {0.0485812, -0.0919804},
        {0.05217, -0.0911264},
        {0.0557163, -0.0909063},
        {0.0600912, -0.052301}} },
      {"Image__2024-08-20__11-35-26.tiff",
       13, {{0.0392268, -0.0198338},
        {0.045665, -0.0597921},
        {0.0492333, -0.0600324},
        {0.0527732, -0.0608319},
        {0.0563161, -0.0598477},
        {0.0598714, -0.0595328},
        {0.0642559, -0.0211625}}},
      {"Image__2024-08-20__11-35-36.tiff",
       14, {{0.0350697, -0.0199392},
        {0.0414537, -0.0599023},
        {0.0450189, -0.0600314},
        {0.0485971, -0.0608293},
        {0.0521036, -0.0599575},
        {0.0556613, -0.0596423},
        {0.0600842, -0.0212697}}},
      {"Image__2024-08-20__11-35-50.tiff",
       15, {{0.0368939, -0.019893},
        {0.0431661, -0.0598575},
        {0.0467305, -0.0598754},
        {0.0503572, -0.0606717},
        {0.0539553, -0.0599092},
        {0.057512, -0.0595942},
        {0.0618721, -0.0212238}}},
      {"Image__2024-08-20__11-36-16.tiff",
       16, {{0.0404535, -0.0217714},
        {0.0467988, -0.0617712},
        {0.0503806, -0.0619017},
        {0.0539366, -0.0627078},
        {0.057495, -0.0618283},
        {0.0610646, -0.0615111},
        {0.0654766, -0.0232118}}},
      {"Image__2024-08-20__11-39-38.tiff",
       17, {{0.0448917, -0.0330553},
        {0.0510227, -0.073134},
        {0.0545932, -0.0731577},
        {0.0581933, -0.0740053},
        {0.0617381, -0.0732059},
        {0.0652898, -0.0727608},
        {0.0705292, -0.0297364}}},
      {"Image__2024-08-20__11-39-50.tiff",
       18, {{0.0440609, -0.0329775},
        {0.0501644, -0.0730391},
        {0.0537369, -0.0731802},
        {0.0573354, -0.0740278},
        {0.0608815, -0.0732283},
        {0.0644394, -0.0729004},
        {0.0697106, -0.0291709}}},
      {"Image__2024-08-20__11-39-57.tiff",
       19, {{0.0397804, -0.0327905},
        {0.0461171, -0.0726764},
        {0.0496828, -0.0726997},
        {0.0532755, -0.0737806},
        {0.056774, -0.0728656},
        {0.0603261, -0.0724208},
        {0.0655835, -0.0275267}}},
      {"Image__2024-08-20__11-40-09.tiff",
       20, {{0.0324099, -0.0463211},
        {0.0388774, -0.0863565},
        {0.0424431, -0.0865087},
        {0.0460186, -0.0874062},
        {0.0495768, -0.0864428},
        {0.053137, -0.0861008},
        {0.0582022, -0.040676}} }
  };

  test_data.joint_properties                   = jp;
  test_data.scanner_config.gray_minimum_top    = 16;
  test_data.scanner_config.gray_minimum_bottom = 16;
  test_data.scanner_config.gray_minimum_wall   = 16;
  test_data.camera_properties                  = camera_properties;
  test_data.images_path                        = "./src/scanner/joint_model/test/test_data/";
  test_data.image_test_data                    = image_test_data;

  test_data_vector.push_back(test_data);
  return test_data_vector;
}

TEST_SUITE("Test Big Snake") {
  TEST_CASE("Parse Images") {
    auto test_data_vector = Setup();

    for (const auto& test_data : test_data_vector) {
      // Map filename -> test data for quick lookup
      std::unordered_map<std::string, ImageTestData> filename_to_test;
      filename_to_test.reserve(test_data.image_test_data.size());
      for (const auto& td : test_data.image_test_data) {
        filename_to_test[td.filename] = td;
      }

      // Only use files that are explicitly listed in image_test_data and exist on disk
      auto image_files = GetImageFiles(test_data.images_path, test_data.image_test_data);

      for (const auto& abs_path : image_files) {
        const std::string filename = abs_path.filename().string();

        SUBCASE(("Processing: " + filename).c_str()) {
          auto it = filename_to_test.find(filename);
          if (it == filename_to_test.end()) {
            LOG_INFO("Skipping {} - no test data found", filename);
            return;  // end subcase
          }
          const auto& td = it->second;

          // Create a new camera model for each test case using the properties
          auto camera_model = std::make_unique<scanner::image::TiltedPerspectiveCamera>(test_data.camera_properties);

          auto big_snake = scanner::joint_model::BigSnake(test_data.joint_properties, test_data.scanner_config,
                                                          std::move(camera_model));

          LOG_TRACE("Reading image from {}", abs_path);
          auto grayscale_image = imread(abs_path.string(), cv::IMREAD_GRAYSCALE);
          REQUIRE_MESSAGE(!grayscale_image.empty(),
                          fmt::format("Failed to read image: {} (absolute: {})", td.filename, abs_path));

          auto maybe_image = scanner::image::ImageBuilder::From(grayscale_image, td.filename, 0).Finalize();
          auto* image      = maybe_image.value().get();

          auto res = big_snake.Parse(*image, {}, {}, false, {});
          CHECK(res.has_value());

          auto parsed   = res.value();
          auto& profile = std::get<0>(parsed);

          LOG_TRACE("Testing image: {} (ID: {})", td.filename, td.image_id);

          // Check all 7 points with tolerances
          const double tol_mm = 0.49;  // tolerance in mm
          for (size_t i = 0; i < 7; ++i) {
            const double dx_mm = (td.expected_points[i].x - profile.points[i].x) * 1000.0;
            const double dy_mm = (td.expected_points[i].y - profile.points[i].y) * 1000.0;
            const double dist  = std::sqrt(dx_mm * dx_mm + dy_mm * dy_mm);
            CHECK_MESSAGE(dist <= tol_mm,
                          "image_id=" << td.filename << ": ABW" << i << " distance " << dist << "mm exceeds tolerance");
          }
        }
      }
    }
  }
}
#endif
