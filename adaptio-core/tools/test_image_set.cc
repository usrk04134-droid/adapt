#include <opencv2/highgui/highgui_c.h>
#include <prometheus/registry.h>

#include <boost/program_options.hpp>
#include <boost/range/numeric.hpp>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <Eigen/Eigen>
#include <filesystem>
#include <fstream>
#include <memory>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <optional>

#include "core/file/yaml.h"
#include "core/image/tilted_perspective_camera.h"
#include "core/logging/application_log.h"
#include "core/scanner/image_logger.h"
#include "core/scanner/image_logger_impl.h"
#include "core/scanner/image_provider/basler_camera.h"
#include "core/scanner/image_provider/simulation/camera_simulation.h"
#include "core/scanner/joint_buffer/circular_joint_buffer.h"
#include "core/scanner/joint_model.h"
#include "core/scanner/joint_model/big_snake.h"
#include "core/scanner/scanner.h"
#include "core/scanner/scanner_calibration_configuration.h"
#include "core/scanner/scanner_impl.h"
#include "core/scanner/tiff_handler_impl.h"

namespace po = boost::program_options;

using core::file::Yaml;
using core::image::PlaneCoordinates;
using core::image::TiltedPerspectiveCamera;
using core::image::TiltedPerspectiveCameraProperties;
using core::image::WorkspaceCoordinates;
using core::scanner::BaslerCamera;
using core::scanner::CameraSimulation;
using core::scanner::CircularJointBuffer;
using core::scanner::JointModelPtr;
using core::scanner::JointProperties;
using core::scanner::ScannerImpl;
using core::scanner::joint_model::BigSnake;
using Eigen::Index;

std::atomic<bool> should_shutdown = false;
int exit_code                     = 0;

void ShutdownTestImages(int signum) {
  LOG_WARNING("Shutting down");
  should_shutdown = true;
  exit_code       = signum;
}

void Exit(int signum) {
  LOG_INFO("Exiting...");
  core::logging::DeinitLogging();
  exit(signum);
}

class ScannerOutputCBImpl : public core::scanner::ScannerOutputCB {
 public:
  void ScannerOutput(const core::joint_tracking::JointSlice& joint_slice,
                     const std::array<core::joint_tracking::Coord, 15>& line, const std::optional<double> area,
                     uint64_t time_stamp) override {};
};

auto getNumberOfImages(std::filesystem::path search_path) -> int {
  int number_images = 0;

  std::vector<std::string> supported_images = {"bmp", "tiff"};

  if (std::filesystem::is_regular_file(search_path)) {
    for (auto& supported_image : supported_images) {
      if (search_path.has_extension() &&
          (search_path.extension().string() == std::string(".").append(supported_image))) {
        number_images++;
      }
    }
    if (number_images == 0) {
      LOG_ERROR("Unrecognized file format");
    }
  } else if (std::filesystem::is_directory(search_path)) {
    for (const auto& entry : std::filesystem::directory_iterator(search_path)) {
      if (std::filesystem::is_directory(entry)) {
        continue;
      }

      const auto& path = entry.path();
      for (auto& supported_image : supported_images) {
        if (path.has_extension() && (path.extension().string() == std::string(".").append(supported_image))) {
          number_images++;
        }
      }
    }
  } else {
    LOG_ERROR("Unrecognized file format");
  }
  return number_images;
}

enum DisplayMode { None, Normal, Cropped };

auto main(int argc, char* argv[]) -> int {
  signal(SIGINT, ShutdownTestImages);
  signal(SIGHUP, ShutdownTestImages);
  signal(SIGTERM, ShutdownTestImages);

  po::options_description desc("Allowed options");
  desc.add_options()("verbose,v", "Sets verbosity to INFO (same as --info)");
  desc.add_options()("info", "Sets verbosity to INFO");
  desc.add_options()("debug", "Sets verbosity to DEBUG");
  desc.add_options()("trace", "Sets verbosity to TRACE");
  desc.add_options()("silent,s", "Disable all output except errors");
  desc.add_options()("show", "Shows the image overlaid with the processed data");
  desc.add_options()("cropped",
                     "Shows the image overlaid with the processed data but cropped to the area around the joint");
  desc.add_options()("images,i", po::value<std::string>(), "Path to input images");
  desc.add_options()("config,c", po::value<std::string>(), "Path to camera configuration file");
  desc.add_options()("joint,j", po::value<std::string>(), "Path to joint geometry file");
  desc.add_options()("sim", "Use simulated image provider");
  desc.add_options()("loop", "Loop over image set");
  desc.add_options()("report-path,r", po::value<std::string>(), "Path to test report");

  po::variables_map map;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), map);
  po::notify(map);

  core::logging::InitLogging();
  core::logging::SetLogLevel(1);

  if (map.count("silent") > 0 || map.count("s") > 0) {
    core::logging::SetLogLevel(-1);
  }

  if (map.count("verbose") > 0 || map.count("v") > 0 || map.count("info") > 0) {
    core::logging::SetLogLevel(1);
  }

  if (map.count("debug") > 0) {
    core::logging::SetLogLevel(2);
  }

  if (map.count("trace") > 0) {
    core::logging::SetLogLevel(3);
  }

  enum DisplayMode display_mode;
  if (map.count("show") > 0) {
    display_mode = DisplayMode::Normal;
  } else if (map.count("cropped") > 0) {
    display_mode = DisplayMode::Cropped;
  } else {
    display_mode = DisplayMode::None;
  }

  std::string configuration_path;
  if (map.count("config") > 0) {
    configuration_path = map["config"].as<std::string>();
  } else if (map.count("c") > 0) {
    configuration_path = map["c"].as<std::string>();
  } else {
    LOG_ERROR("No configuration supplied");
    Exit(1);
  }
  LOG_INFO("Using configuration from path {}", configuration_path);
  auto maybe_scanner_cfg = Yaml::FromFile(configuration_path, "camera");

  if (maybe_scanner_cfg.has_error()) {
    LOG_ERROR("Failed to read configuration file: {}", maybe_scanner_cfg.error().to_string());
    Exit(1);
  }
  auto scanner_cfg = maybe_scanner_cfg.value()->AsUnorderedMap();

  std::unique_ptr<core::scanner::ImageProvider> image_provider;

  auto number_images = 0;
  bool loop          = false;
  core::scanner::Fov fov;
  fov.width    = 3500;
  fov.height   = 2500;
  fov.offset_x = 312;
  fov.offset_y = 0;

  if (map.count("sim") > 0 || map.count("images") > 0) {
    LOG_INFO("Using simulated image provider");
    if (map.count("images") == 0) {
      LOG_ERROR("No image supplied");
      Exit(1);
    }

    if (map.count("loop") > 0) {
      loop = true;
    }

    auto image_path = map["images"].as<std::string>();

    number_images = getNumberOfImages(image_path);

    LOG_INFO("Using image(s) from path: {} number of: images {} loop mode: {}", image_path, number_images, loop);

    core::scanner::SimConfig sim_config;
    sim_config.realtime    = false;
    sim_config.images_path = image_path;

    image_provider = std::make_unique<CameraSimulation>(sim_config, loop);
  } else {
    number_images = 2000;

    core::scanner::BaslerConfig bc;
    bc.exposure_time = 5000.0;
    bc.gain          = 12.0;
    image_provider   = std::make_unique<BaslerCamera>(bc, fov, nullptr);
  }

  std::string joint_geo_path;
  if (map.count("joint") > 0) {
    joint_geo_path = map["joint"].as<std::string>();
  } else if (map.count("j") > 0) {
    joint_geo_path = map["j"].as<std::string>();
  } else {
    LOG_ERROR("No joint geometry supplied");
    Exit(1);
  }

  std::string report_path;
  if (map.count("report-path") > 0) {
    report_path = map["report-path"].as<std::string>();
  } else if (map.count("r") > 0) {
    report_path = map["r"].as<std::string>();
  }

  LOG_INFO("Using joint geometry from path {}", joint_geo_path);
  auto maybe_joint_geo = Yaml::FromFile(joint_geo_path, "joint");

  if (maybe_joint_geo.has_error()) {
    LOG_ERROR("Failed to read joint geometry file: {}", maybe_joint_geo.error().to_string());
    Exit(1);
  }

  auto camera_properties       = TiltedPerspectiveCameraProperties::FromUnorderedMap(scanner_cfg);
  camera_properties.config_fov = fov;

  auto camera_model            = std::make_unique<TiltedPerspectiveCamera>(camera_properties);
  const auto* camera_model_raw = camera_model.get();

  auto joint_buffer            = std::make_unique<CircularJointBuffer>();
  const auto* joint_buffer_raw = joint_buffer.get();

  auto joint_geo_map = maybe_joint_geo.value()->AsUnorderedMap();
  // Properties for images in test/assets/v-joint
  const JointProperties properties = {
      .upper_joint_width           = joint_geo_map.at("joint/upper_joint_width").Value<double>().value(),
      .left_max_surface_angle      = joint_geo_map.at("joint/left_max_surface_angle").Value<double>().value(),
      .right_max_surface_angle     = joint_geo_map.at("joint/right_max_surface_angle").Value<double>().value(),
      .left_joint_angle            = joint_geo_map.at("joint/left_joint_angle").Value<double>().value(),
      .right_joint_angle           = joint_geo_map.at("joint/right_joint_angle").Value<double>().value(),
      .groove_depth                = joint_geo_map.at("joint/groove_depth").Value<double>().value(),
      .upper_joint_width_tolerance = 7.0,
      .surface_angle_tolerance     = 10.0 * static_cast<double>(EIGEN_PI) / 180.0,
      .groove_angle_tolerance      = 9.0 * static_cast<double>(EIGEN_PI) / 180.0,
      .offset_distance             = 3.0,
  };

  JointModelPtr joint_model = JointModelPtr(new BigSnake(properties, {48, 24, 48}, std::move(camera_model)));

  ScannerOutputCBImpl scanner_output_cb;

  std::stringstream test_report;
  test_report << "data:" << std::endl;

  auto tiff_handler = std::make_unique<core::scanner::TiffHandlerImpl>();
  auto image_logger = core::scanner::ImageLoggerImpl(std::move(tiff_handler));

  // Uncomment to test image logging
  // image_logger.Start({.type=core::scanner::LoggerType::DIRECT, .base_log_path=image_path, .sample_rate=1,
  // .buffer_size=1});
  auto registry = std::make_shared<prometheus::Registry>();
  auto scanner  = ScannerImpl(
      image_provider.get(), std::move(joint_buffer), [](bool state) {}, &scanner_output_cb, std::move(joint_model),
      &image_logger, registry.get());

  auto result = scanner.Start(core::scanner::ScannerSensitivity::NORMAL, true);
  if (!result) {
    LOG_ERROR("Unable to start scanner: {}", result.error().what());
    return 1;
  }

  if (display_mode != DisplayMode::None) {
    cv::namedWindow("output", cv::WINDOW_NORMAL);
  }

  uint64_t max_processing_time = 0;
  std::vector<std::uint64_t> processing_times;
  int i = 0;
  while (!should_shutdown) {
    if ((scanner.CountOfReceivedImages() >= number_images) && !loop) {
      should_shutdown = true;
    }

    boost::this_thread::sleep(boost::posix_time::milliseconds(100));

    // scanner.Update();

    if (display_mode != DisplayMode::None && cvGetWindowHandle("output") == nullptr) {
      ShutdownTestImages(0);
      break;
    }

    // We disregard the normal callback into ScannerOutput. Instead we directly look at the joint_buffer
    // that we passed to the ScannerImpl object.
    auto maybe_slice = joint_buffer_raw->GetSlice();

    if (maybe_slice.has_value()) {
      auto slice = maybe_slice.value();

      if (slice.processing_time > max_processing_time) {
        max_processing_time = slice.processing_time;
      }

      processing_times.push_back(slice.processing_time);

      // Add abw points to test report
      test_report << "  ABWPoints" << i << ":" << std::endl;
      i++;
      int point_index = 0;
      for (auto abw : slice.profile.points) {
        test_report << "    ABW" << point_index << ":" << std::endl;
        test_report << std::setprecision(std::numeric_limits<double>::digits10 + 1) << "      x: " << abw.x
                    << std::endl;
        test_report << std::setprecision(std::numeric_limits<double>::digits10 + 1) << "      y: " << abw.y
                    << std::endl;
        point_index++;
      }
      test_report << "    image: " << slice.image_name << std::endl;

      if (display_mode == DisplayMode::None) {
        continue;
      }
      LOG_TRACE("Showing image");
      cv::Mat cv_image;
      cv::Mat cv_image_color;
      cv::eigen2cv(slice.image_data.value(), cv_image);
      cv::cvtColor(cv_image, cv_image_color, cv::COLOR_GRAY2RGBA);

      auto maybe_centroids = camera_model_raw->WorkspaceToImage(slice.centroids, slice.vertical_crop_start);

      if (maybe_centroids.has_value()) {
        auto centroids = maybe_centroids.value();

        for (auto centroid : centroids.colwise()) {
          cv::circle(cv_image_color, cv::Point(centroid(0), centroid(1)), 1, cv::Scalar(255, 0, 0), cv::LINE_8);
        }
      }

      // int alternate = 0;

      int min_x = INT_MAX;
      int max_x = INT_MIN;
      int min_y = INT_MAX;
      int max_y = INT_MIN;
      for (auto edge : slice.profile.points) {
        WorkspaceCoordinates wcs(3, 1);
        wcs << edge.x, edge.y, 0.0;
        auto img = camera_model_raw->WorkspaceToImage(wcs, slice.vertical_crop_start).value();
        min_x    = std::min(static_cast<int>(img(0, 0)), min_x);
        min_y    = std::min(static_cast<int>(img(1, 0)), min_y);
        max_x    = std::max(static_cast<int>(img(0, 0)), max_x);
        max_y    = std::max(static_cast<int>(img(1, 0)), max_y);

        cv::circle(cv_image_color, cv::Point(img(0, 0), img(1, 0)), 10, cv::Scalar(0, 255, 255), 10);
      }

      LOG_TRACE("OpenCV Mat size: {}x{}", cv_image.cols, cv_image.rows);
      if (display_mode == DisplayMode::Cropped) {
        const int margin = 200;
        min_x            = std::max(0, min_x - margin);
        min_y            = std::max(0, min_y - margin);
        max_x            = std::min(cv_image.cols - 1, max_x + margin);
        max_y            = std::min(cv_image.rows - 1, max_y + margin);
        cv::putText(cv_image_color, slice.image_name, cv::Point(min_x + 10, min_y + 30), cv::FONT_HERSHEY_DUPLEX, 1.0,
                    cv::Scalar(0, 255, 255), 2);
        cv::imshow("output", cv_image_color(cv::Range(min_y, max_y), cv::Range(min_x, max_x)));
        // cv::imwrite("out.tiff", cv_image_color(cv::Range(min_y, max_y), cv::Range(min_x, max_x)));
      } else {
        cv::imshow("output", cv_image_color);
      }

      cv::waitKey(50);
    }
  }

  if (processing_times.size() != 0) {
    auto joint_geo    = maybe_joint_geo.value()->ToString(joint_geo_map);
    auto scanner_data = maybe_scanner_cfg.value()->ToString(maybe_scanner_cfg.value()->AsUnorderedMap());

    uint64_t sum = 0;
    for (const auto& n : processing_times) {
      sum += n;
    }

    test_report << std::endl
                << "header:"
                << std::endl
                //<< "image_dir: " << map["images"].as<std::string>() << std::endl
                << "image_processing_time: \n  max: " << max_processing_time
                << "\n  mean: " << sum / processing_times.size() << std::endl
                << "joint_geometry:" << std::endl
                << joint_geo.value() << std::endl
                << "scanner_config:" << std::endl
                << scanner_data.value() << std::endl;
    std::string out_report = test_report.str();

    const std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    const std::time_t t_c                                        = std::chrono::system_clock::to_time_t(now);

    if (report_path.empty()) {
      if (map.count("images") > 0) {
        report_path = map["images"].as<std::string>();
      }
    } else {
      std::filesystem::path dir(report_path);

      if (!(std::filesystem::exists(dir))) {
        if (std::filesystem::create_directory(dir)) {
          LOG_INFO("Created dir where test report will be stored: {}", report_path);
        }
      }
    }

    if (!report_path.empty()) {
      std::stringstream f_name;
      f_name << report_path << std::put_time(std::localtime(&t_c), "/%F_%T_ABW_calc_report_cpp.yaml");
      std::ofstream file(f_name.str(), std::ofstream::out);
      file << out_report;
      file.close();
    }
  }

  if (display_mode != DisplayMode::None) {
    cv::waitKey(50000);
    cv::destroyAllWindows();
  }

  scanner.Stop();

  Exit(exit_code);
}
