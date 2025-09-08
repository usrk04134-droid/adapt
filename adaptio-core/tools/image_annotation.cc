#include <boost/program_options.hpp>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <Eigen/Eigen>
#include <filesystem>
#include <fstream>
#include <optional>

#include "core/file/yaml.h"
#include "core/image/tilted_perspective_camera.h"
#include "core/logging/application_log.h"

namespace po = boost::program_options;

using core::file::Yaml;
using core::image::PlaneCoordinates;
using core::image::TiltedPerspectiveCamera;
using core::image::TiltedPerspectiveCameraProperties;
using core::image::WorkspaceCoordinates;
using Eigen::Index;
using Eigen::Matrix;

std::atomic<bool> shutdown = false;
int exit_code              = 0;

void Shutdown(int signum) {
  LOG_WARNING("Shutting down");
  shutdown  = true;
  exit_code = signum;
}

void Exit(int signum) {
  LOG_INFO("Exiting...");
  core::logging::DeinitLogging();
  exit(signum);
}

auto main(int argc, char* argv[]) -> int {
  signal(SIGINT, Shutdown);
  signal(SIGHUP, Shutdown);
  signal(SIGTERM, Shutdown);

  // Coordinates are read from stdin
  // image file name is given on
  // file name
  //

  po::options_description desc("Allowed options");
  desc.add_options()("verbose,v", "Sets verbosity to INFO (same as --info)");
  desc.add_options()("info", "Sets verbosity to INFO");
  desc.add_options()("debug", "Sets verbosity to DEBUG");
  desc.add_options()("trace", "Sets verbosity to TRACE");
  desc.add_options()("reverse", "Converts points from laser plane to pixels");
  desc.add_options()("config,c", po::value<std::string>(), "Path to camera configuration file");

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

  bool reverse = map.count("reverse") > 0;

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

  auto scanner_cfg_map = maybe_scanner_cfg.value()->AsUnorderedMap();

  // ToDo: Should be read from config file
  scanner_cfg_map.emplace("camera/extrinsic/offset/x", 0);
  scanner_cfg_map.emplace("camera/extrinsic/offset/y", 0);

  auto camera_properties = TiltedPerspectiveCameraProperties::FromUnorderedMap(scanner_cfg_map);

  camera_properties.config_fov.offset_x = 312;
  camera_properties.config_fov.offset_y = 0;

  auto camera_model = std::make_unique<TiltedPerspectiveCamera>(camera_properties);

  if (!reverse) {
    // Create 2xN matrix from stdin
    auto number_of_abw_points = 7;
    Matrix<double, 2, Eigen::Dynamic, Eigen::RowMajor> coordinates(2, number_of_abw_points);
    for (int i = 0; i < number_of_abw_points; i++) {
      std::cin >> coordinates(0, i) >> coordinates(1, i);
    }

    std::cout << "  input_xs: [" << coordinates(0, 0) << "," << coordinates(0, 1) << "," << coordinates(0, 2) << ","
              << coordinates(0, 3) << "," << coordinates(0, 4) << "," << coordinates(0, 5) << "," << coordinates(0, 6)
              << "]" << std::endl
              << "  input_ys: [" << coordinates(1, 0) << "," << coordinates(1, 1) << "," << coordinates(1, 2) << ","
              << coordinates(1, 3) << "," << coordinates(1, 4) << "," << coordinates(1, 5) << "," << coordinates(1, 6)
              << "]" << std::endl;

    auto maybe_wcs_coordinates = camera_model->ImageToWorkspace(coordinates, 0);

    if (!maybe_wcs_coordinates.has_value()) {
      LOG_ERROR("Failed to convert coordinates to LPCS.");
      Exit(1);
    }

    auto wcs_coordinates = maybe_wcs_coordinates.value();

    std::cout << "  abw_points_xs: [" << wcs_coordinates(0, 0) << "," << wcs_coordinates(0, 1) << ","
              << wcs_coordinates(0, 2) << "," << wcs_coordinates(0, 3) << "," << wcs_coordinates(0, 4) << ","
              << wcs_coordinates(0, 5) << "," << wcs_coordinates(0, 6) << "]" << std::endl
              << "  abw_points_ys: [" << wcs_coordinates(1, 0) << "," << wcs_coordinates(1, 1) << ","
              << wcs_coordinates(1, 2) << "," << wcs_coordinates(1, 3) << "," << wcs_coordinates(1, 4) << ","
              << wcs_coordinates(1, 5) << "," << wcs_coordinates(1, 6) << "]" << std::endl;

    // The y coordinates of P1 and P5 are specified in such a way that they cannot be determined
    // without knowing how pixel coordinates relate to LPCS.
    // After converting, we translate P0, P1 3 mm to the right, and P5, p6 3 mm to the left and
    // convert back to pixel coordinates.

    wcs_coordinates(0, 0) += 0.003;
    wcs_coordinates(0, 1) += 0.003;
    wcs_coordinates(0, 5) -= 0.003;
    wcs_coordinates(0, 6) -= 0.003;

    auto maybe_translated = camera_model->WorkspaceToImage(wcs_coordinates, 0);

    if (!maybe_translated.has_value()) {
      LOG_ERROR("Failed to convert translated coordinates back from LPCS.");
      Exit(1);
    }

    auto translated = maybe_translated.value();

    std::cout << "# translation by 3.0 mm means this many pixels:" << std::endl
              << "# P0: " << translated(0, 0) - coordinates(0, 0) << std::endl
              << "# P1: " << translated(0, 1) - coordinates(0, 1) << std::endl
              << "# P5: " << translated(0, 5) - coordinates(0, 5) << std::endl
              << "# P6: " << translated(0, 6) - coordinates(0, 6) << std::endl;
  } else {
    // Create 2xN matrix from stdin
    auto number_of_abw_points = 7;
    Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor> coordinates(3, number_of_abw_points);
    for (int i = 0; i < number_of_abw_points; i++) {
      std::cin >> coordinates(0, i) >> coordinates(1, i);
    }

    std::cout << "  input_xs: [" << coordinates(0, 0) << "," << coordinates(0, 1) << "," << coordinates(0, 2) << ","
              << coordinates(0, 3) << "," << coordinates(0, 4) << "," << coordinates(0, 5) << "," << coordinates(0, 6)
              << "]" << std::endl
              << "  input_ys: [" << coordinates(1, 0) << "," << coordinates(1, 1) << "," << coordinates(1, 2) << ","
              << coordinates(1, 3) << "," << coordinates(1, 4) << "," << coordinates(1, 5) << "," << coordinates(1, 6)
              << "]" << std::endl;

    auto maybe_image_coordinates = camera_model->WorkspaceToImage(coordinates, 0);

    if (!maybe_image_coordinates.has_value()) {
      LOG_ERROR("Failed to convert coordinates to LPCS.");
      Exit(1);
    }

    auto image_coordinates = maybe_image_coordinates.value();

    std::cout << "  image_xs: [" << image_coordinates(0, 0) << "," << image_coordinates(0, 1) << ","
              << image_coordinates(0, 2) << "," << image_coordinates(0, 3) << "," << image_coordinates(0, 4) << ","
              << image_coordinates(0, 5) << "," << image_coordinates(0, 6) << "]" << std::endl
              << "  image_ys: [" << image_coordinates(1, 0) << "," << image_coordinates(1, 1) << ","
              << image_coordinates(1, 2) << "," << image_coordinates(1, 3) << "," << image_coordinates(1, 4) << ","
              << image_coordinates(1, 5) << "," << image_coordinates(1, 6) << "]" << std::endl;
  }

  Exit(exit_code);
}
