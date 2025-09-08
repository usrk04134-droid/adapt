#include <yaml-cpp/yaml.h>

#include <atomic>
#include <boost/program_options.hpp>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <ranges>
#include <utility>

namespace po = boost::program_options;

std::atomic<bool> shutdown = false;
int exit_code              = 0;

void Shutdown(int signum) {
  shutdown  = true;
  exit_code = signum;
}

void Exit(int signum) { exit(signum); }

struct Offset {
  int x;
  int y;
};

struct Image {
  int id;
  std::filesystem::path file_path;
  int joint_id;
  int scanner_id;
  Offset offset;
};

struct Scanner {
  std::filesystem::path file_path;
  std::string serial_number;
};

struct Joint {
  std::filesystem::path file_path;
};

struct AbwPoint {
  float x;
  float y;
};

struct Annotation {
  int id;
  std::vector<float> xs;
  std::vector<float> ys;
};

auto GetImages(const YAML::Node& data_set) -> std::vector<Image> {
  std::vector<Image> images;

  auto node_images = data_set["images"];

  for (YAML::const_iterator it = node_images.begin(); it != node_images.end(); ++it) {
    Image image;
    image.id         = (*it)["id"].as<int>();
    image.file_path  = (*it)["file_path"].as<std::string>();
    image.joint_id   = (*it)["joint_id"].as<int>();
    image.scanner_id = (*it)["scanner_id"].as<int>();

    images.push_back(image);
  }

  return images;
}

auto GetScanners(const YAML::Node& data_set) -> std::map<int, Scanner> {
  std::map<int, Scanner> scanners;

  auto node_scanners = data_set["scanners"];

  for (YAML::const_iterator it = node_scanners.begin(); it != node_scanners.end(); ++it) {
    Scanner scanner;
    auto id               = (*it)["id"].as<int>();
    scanner.file_path     = (*it)["file_path"].as<std::string>();
    scanner.serial_number = (*it)["serial_number"].as<std::string>();

    scanners.insert({id, scanner});
  }
  return scanners;
}

auto GetJoints(const YAML::Node& data_set) -> std::map<int, Joint> {
  std::map<int, Joint> joints;
  auto node_joints = data_set["joints"];

  for (YAML::const_iterator it = node_joints.begin(); it != node_joints.end(); ++it) {
    Joint joint;
    auto id         = (*it)["id"].as<int>();
    joint.file_path = (*it)["file_path"].as<std::string>();

    joints.insert({id, joint});
  }

  return joints;
}

auto GetAnnotations(const YAML::Node& data_set) -> std::map<int, Annotation> {
  // key = image_id
  std::map<int, Annotation> annotations;

  auto node_joints = data_set["annotations"];

  for (YAML::const_iterator it = node_joints.begin(); it != node_joints.end(); ++it) {
    Annotation annotation;
    auto id       = (*it)["image_id"].as<int>();
    annotation.id = (*it)["id"].as<int>();
    annotation.xs = (*it)["abw_points_xs"].as<std::vector<float>>();
    annotation.ys = (*it)["abw_points_ys"].as<std::vector<float>>();

    annotations.insert({id, annotation});
  }

  return annotations;
}

auto GetAnnotatedAbwPoints(const YAML::Node& annotations) -> std::vector<AbwPoint> {
  std::vector<AbwPoint> abws;
  auto abw_points = annotations["data"];

  abws.push_back(
      AbwPoint{abw_points["ABWPoints0"]["ABW0"]["x"].as<float>(), abw_points["ABWPoints0"]["ABW0"]["y"].as<float>()});
  abws.push_back(
      AbwPoint{abw_points["ABWPoints0"]["ABW1"]["x"].as<float>(), abw_points["ABWPoints0"]["ABW1"]["y"].as<float>()});
  abws.push_back(
      AbwPoint{abw_points["ABWPoints0"]["ABW2"]["x"].as<float>(), abw_points["ABWPoints0"]["ABW2"]["y"].as<float>()});
  abws.push_back(
      AbwPoint{abw_points["ABWPoints0"]["ABW3"]["x"].as<float>(), abw_points["ABWPoints0"]["ABW3"]["y"].as<float>()});
  abws.push_back(
      AbwPoint{abw_points["ABWPoints0"]["ABW4"]["x"].as<float>(), abw_points["ABWPoints0"]["ABW4"]["y"].as<float>()});
  abws.push_back(
      AbwPoint{abw_points["ABWPoints0"]["ABW5"]["x"].as<float>(), abw_points["ABWPoints0"]["ABW5"]["y"].as<float>()});
  abws.push_back(
      AbwPoint{abw_points["ABWPoints0"]["ABW6"]["x"].as<float>(), abw_points["ABWPoints0"]["ABW6"]["y"].as<float>()});

  return abws;
}

auto CheckResult(const Annotation& annotation, const std::vector<AbwPoint>& annotated_abws, float diff) -> bool {
  if (annotation.xs.size() != annotated_abws.size()) {
    std::cout << "Number of ABW points differ: " << annotation.xs.size() << " vs " << annotated_abws.size() << "\n";
    return false;
  }

  bool within_limit = true;
  std::stringstream diff_string;
  diff_string << "Diff (mm): ";
  for (int i = 0; i < annotated_abws.size(); i++) {
    auto dx = annotation.xs[i] * 1000 - annotated_abws[i].x * 1000;
    auto dy = annotation.ys[i] * 1000 - annotated_abws[i].y * 1000;

    auto dist = std::sqrt(dx * dx + dy * dy);
    diff_string << "ABW(" << i << "): {" << dist << "} ";

    if (dist > diff) {
      std::cout << "Diff on ABW point " << i << " too large\n";
      within_limit = false;
    }
  }

  std::cout << diff_string.str() << "\n";

  if (!within_limit) {
    for (int i = 0; i < annotation.xs.size(); i++) {
      std::cout << annotation.xs[i] << std::endl << annotation.ys[i] << std::endl;
    }
  }

  return within_limit;
}

auto GetProgramOptions(const po::variables_map& map, const po::options_description& desc)
    -> std::tuple<std::filesystem::path, float, std::filesystem::path, std::optional<uint64_t>> {
  if (map.count("help") > 0 || map.count("h") > 0) {
    std::cout << desc << "\n";
  }

  std::filesystem::path data_set_path;
  if (map.count("dataset") > 0) {
    data_set_path = map["dataset"].as<std::filesystem::path>();
  } else {
    std::cout << "No data set supplied\n";
    Exit(1);
  }

  float diff = 0.0;
  if (map.count("diff") > 0) {
    diff = map["diff"].as<float>();
  } else {
    std::cout << "Max allowed diff is not supplied\n";
    Exit(1);
  }

  std::filesystem::path test_app_path = "./build/test-images";
  if (map.count("test-app") > 0) {
    test_app_path = map["test-app"].as<std::filesystem::path>();
  }

  std::optional<uint64_t> processing_time_limit;
  if (map.count("processing_time_limit") > 0) {
    processing_time_limit = map["processing_time_limit"].as<uint64_t>();
  } else if (map.count("p") > 0) {
    processing_time_limit = map["p"].as<uint64_t>();
  }

  return {data_set_path, diff, test_app_path, processing_time_limit};
}

auto main(int argc, char* argv[]) -> int {
  signal(SIGINT, Shutdown);
  signal(SIGHUP, Shutdown);
  signal(SIGTERM, Shutdown);

  po::options_description desc("Allowed options");
  desc.add_options()("help,h", "Shows help");
  desc.add_options()("dataset", po::value<std::filesystem::path>(), "Path to data set");
  desc.add_options()("diff", po::value<float>(), "Max allowed ABW point difference in mm");
  desc.add_options()("test-app", po::value<std::filesystem::path>(), "Path to test application. Optional");
  desc.add_options()("processing_time_limit,p", po::value<uint64_t>(), "Max allowed processing time in ms. Optional");

  po::variables_map map;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), map);
  po::notify(map);

  auto [data_set_path, diff, test_app_path, process_limit] = GetProgramOptions(map, desc);

  // Parse data set file
  YAML::Node data_set = YAML::LoadFile(data_set_path);

  auto images      = GetImages(data_set);
  auto scanners    = GetScanners(data_set);
  auto joints      = GetJoints(data_set);
  auto annotations = GetAnnotations(data_set);

  for (const auto& image : images) {
    // Run test
    std::string output_path = std::tmpnam(nullptr);

    auto joint   = joints[image.joint_id];
    auto scanner = scanners[image.scanner_id];

    auto data_set_dir = data_set_path.parent_path();
    std::stringstream cmd_stream;
    cmd_stream << test_app_path << " --images " << data_set_dir << "/" << image.file_path << " --joint " << data_set_dir
               << "/" << joint.file_path << " --config " << data_set_dir << "/" << scanner.file_path
               << " --report-path " << output_path;

    std::cout << cmd_stream.str() << std::endl;
    std::cout << std::flush;
    auto ret = std::system(cmd_stream.str().c_str());
    if (0 != ret) {
      std::cout << "Not able to run test-image, exit code: " << ret << "\n";
      std::cout << cmd_stream.str() << std::endl;
      return 1;
    }

    // Evaluate
    std::vector<std::filesystem::path> report_file;
    std::ranges::transform(std::filesystem::directory_iterator(output_path), back_inserter(report_file),
                           [](const auto& dir_file) { return dir_file.path(); });

    if (report_file.size() != 1) {
      return 1;
    }

    YAML::Node annotation = YAML::LoadFile(report_file[0]);

    std::filesystem::remove_all(output_path);
    auto annotated_abw = GetAnnotatedAbwPoints(annotation);

    // Check max allowed diff for ABW points
    if (!CheckResult(annotations[image.id], annotated_abw, diff)) {
      std::cout << "Test failed for image id:" << image.id << "\n";
      return 1;
    }

    // Check max allowed image processing time
    auto mean_image_processing_time = annotation["image_processing_time"]["mean"].as<uint64_t>();

    if (process_limit.has_value()) {
      if (mean_image_processing_time > process_limit.value()) {
        std::cout << "Mean image processing time (" << mean_image_processing_time << ") is above limit ("
                  << process_limit.value() << ") for image Id (" << image.id << ")\n";
        return 1;
      }
    }
  }

  std::cout << "OK!\n";
  return 0;
}
