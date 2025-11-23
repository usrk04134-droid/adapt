#include <opencv2/highgui/highgui_c.h>
#include <prometheus/registry.h>
#include <signal.h>

#include <algorithm>
#include <atomic>
#include <boost/program_options.hpp>
#include <boost/range/numeric.hpp>
#include <chrono>
#include <climits>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <Eigen/Eigen>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <numbers>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "common/file/yaml.h"
#include "common/groove/groove.h"
#include "common/logging/application_log.h"
#include "scanner/core/scanner_types.h"
#include "scanner/core/src/scanner.h"
#include "scanner/core/src/scanner_impl.h"
#include "scanner/image/camera_model.h"
#include "scanner/image/tiff_handler_impl.h"
#include "scanner/image/tilted_perspective_camera.h"
#include "scanner/image_logger/src/image_logger_impl.h"
#include "scanner/image_provider/image_provider.h"
#include "scanner/image_provider/image_provider_configuration.h"
#include "scanner/image_provider/src/basler_camera.h"
#include "scanner/image_provider/src/camera_simulation.h"
#include "scanner/joint_buffer/circular_joint_buffer.h"
#include "scanner/joint_model/joint_model.h"
#include "scanner/joint_model/src/big_snake.h"
#include "scanner/slice_provider/slice_provider.h"
#include "scanner/slice_provider/src/slice_provider_impl.h"

namespace po = boost::program_options;

using common::file::Yaml;
using Eigen::Index;
using scanner::ScannerImpl;
using scanner::image::PlaneCoordinates;
using scanner::image::TiltedPerspectiveCamera;
using scanner::image::TiltedPerspectiveCameraProperties;
using scanner::image::WorkspaceCoordinates;
using scanner::image_provider::BaslerCamera;
using scanner::image_provider::CameraSimulation;
using scanner::joint_buffer::CircularJointBuffer;
using scanner::joint_model::BigSnake;
using scanner::joint_model::JointModelPtr;
using scanner::joint_model::JointProperties;
using scanner::slice_provider::SliceConfidence;

std::atomic<bool> should_shutdown = false;
int exit_code                     = 0;

void ShutdownTestImages(int signum) {
  LOG_WARNING("Shutting down");
  should_shutdown = true;
  exit_code       = signum;
}

void Exit(int signum) {
  LOG_INFO("Exiting...");
  common::logging::DeinitLogging();
  exit(signum);
}

class ScannerOutputCBImpl : public scanner::ScannerOutputCB {
 public:
  void ScannerOutput(const common::Groove& groove, const std::array<common::Point, 100>& profile, uint64_t time_stamp,
                     SliceConfidence confidence) override {};
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

  common::logging::InitLogging();
  common::logging::SetLogLevel(1);

  if (map.count("silent") > 0 || map.count("s") > 0) {
    common::logging::SetLogLevel(-1);
  }

  if (map.count("verbose") > 0 || map.count("v") > 0 || map.count("info") > 0) {
    common::logging::SetLogLevel(1);
  }

  if (map.count("debug") > 0) {
    common::logging::SetLogLevel(2);
  }

  if (map.count("trace") > 0) {
    common::logging::SetLogLevel(3);
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

  std::unique_ptr<scanner::image_provider::ImageProvider> image_provider;

  auto number_images = 0;
  bool loop          = false;
  scanner::image_provider::Fov fov;
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

    scanner::image_provider::SimConfig sim_config;
    sim_config.realtime    = false;
    sim_config.images_path = image_path;

    image_provider = std::make_unique<CameraSimulation>(sim_config, loop);
  } else {
    number_images = 2000;

    scanner::image_provider::BaslerConfig bc;
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

  auto joint_buffer          = std::make_unique<CircularJointBuffer>();
  auto steady_clock_now_func = []() { return std::chrono::steady_clock::now(); };
  auto slice_provider =
      std::make_unique<scanner::slice_provider::SliceProviderImpl>(std::move(joint_buffer), steady_clock_now_func);
  auto* slice_provider_raw = slice_provider.get();
  auto joint_geo_map       = maybe_joint_geo.value()->AsUnorderedMap();
  // Properties for images in test/assets/v-joint
  const JointProperties properties = {
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

  JointModelPtr joint_model = JointModelPtr(new BigSnake(properties, {48, 16, 48}, std::move(camera_model)));

  ScannerOutputCBImpl scanner_output_cb;

  std::stringstream test_report;
  test_report << "data:" << std::endl;

  auto tiff_handler = std::make_unique<scanner::image::TiffHandlerImpl>();
  auto image_logger = scanner::image_logger::ImageLoggerImpl(std::move(tiff_handler));

  // Uncomment to test image logging
  // image_logger.Start({.type=scanner::LoggerType::DIRECT, .base_log_path=image_path, .sample_rate=1,
  // .buffer_size=1});
  auto registry = std::make_shared<prometheus::Registry>();
  auto scanner  = ScannerImpl(
      image_provider.get(), std::move(slice_provider), [](bool state) {}, &scanner_output_cb, std::move(joint_model),
      &image_logger, registry.get());

  auto result = scanner.Start(scanner::ScannerSensitivity::NORMAL, true);
  if (!result) {
    LOG_ERROR("Unable to start scanner: {}", result.error().what());
    return 1;
  }

  if (display_mode != DisplayMode::None) {
    cv::namedWindow("output", cv::WINDOW_NORMAL);
  }

  // auto updated_properties = properties;
  // updated_properties.upper_joint_width_tolerance = 4.;

  // scanner.UpdateJointHistory(updated_properties, {0.,0.});

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
    auto maybe_slice = slice_provider_raw->GetLatestSlice();

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
      for (auto abw : slice.profile.groove) {
        test_report << "    ABW" << point_index << ":" << std::endl;
        test_report << std::setprecision(std::numeric_limits<double>::digits10 + 1) << "      x: " << abw.horizontal
                    << std::endl;
        test_report << std::setprecision(std::numeric_limits<double>::digits10 + 1) << "      y: " << abw.vertical
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
      for (auto edge : slice.profile.groove) {
        WorkspaceCoordinates wcs(3, 1);
        wcs << edge.horizontal, edge.vertical, 0.0;
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
