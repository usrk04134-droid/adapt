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

struct AbwPoint {
  float x;
  float y;
};

auto CheckResult(const std::vector<AbwPoint>& abws, const std::vector<AbwPoint>& previous_abws, float diff) -> bool {
  if (abws.size() != previous_abws.size()) {
    std::cout << "Number of ABW points differ: " << abws.size() << " vs " << previous_abws.size() << "\n";
    return false;
  }

  bool within_limit = true;
  std::stringstream diff_string;
  diff_string << "Diff (mm): ";
  for (int i = 0; i < abws.size(); i++) {
    auto dx = abws[i].x * 1000 - previous_abws[i].x * 1000;
    auto dy = abws[i].y * 1000 - previous_abws[i].y * 1000;

    auto dist = std::sqrt(dx * dx + dy * dy);
    diff_string << "ABW(" << i << "): {" << dist << "} ";

    if (dist > diff) {
      // std::cout << "Diff on ABW point " << i << " too large\n";
      within_limit = false;
    }
  }

  if (!within_limit) {
    std::cout << diff_string.str() << "\n";
  }

  return within_limit;
}

auto GetProgramOptions(const po::variables_map& map, const po::options_description& desc)
    -> std::tuple<std::filesystem::path, float> {
  if (map.count("help") > 0 || map.count("h") > 0) {
    std::cout << desc << "\n";
  }

  std::filesystem::path report_path;
  if (map.count("report") > 0) {
    report_path = map["report"].as<std::filesystem::path>();
  }

  float diff = 0.0;
  if (map.count("diff") > 0) {
    diff = map["diff"].as<float>();
  } else {
    std::cout << "Max allowed diff is not supplied\n";
    Exit(1);
  }

  return {report_path, diff};
}

auto main(int argc, char* argv[]) -> int {
  signal(SIGINT, Shutdown);
  signal(SIGHUP, Shutdown);
  signal(SIGTERM, Shutdown);

  po::options_description desc("Allowed options");
  desc.add_options()("help,h", "Shows help");
  desc.add_options()("report,r", po::value<std::filesystem::path>(), "Path to test report");
  desc.add_options()("diff,d", po::value<float>(), "Max allowed ABW point difference in mm");

  po::variables_map map;
  po::store(po::command_line_parser(argc, argv).options(desc).run(), map);
  po::notify(map);

  auto [report_path, diff] = GetProgramOptions(map, desc);

  std::vector<AbwPoint> previous_abws;
  std::string previous_image;

  YAML::Node report_data = YAML::LoadFile(report_path);

  auto abw_points = report_data["data"];

  for (int i = 0; i < abw_points.size(); i++) {
    std::stringstream cmd_stream;
    cmd_stream << "ABWPoints" << i;
    std::vector<AbwPoint> points;
    points.push_back(
        {abw_points[cmd_stream.str()]["ABW0"]["x"].as<float>(), abw_points[cmd_stream.str()]["ABW0"]["y"].as<float>()});
    points.push_back(
        {abw_points[cmd_stream.str()]["ABW1"]["x"].as<float>(), abw_points[cmd_stream.str()]["ABW1"]["y"].as<float>()});
    points.push_back(
        {abw_points[cmd_stream.str()]["ABW2"]["x"].as<float>(), abw_points[cmd_stream.str()]["ABW2"]["y"].as<float>()});
    points.push_back(
        {abw_points[cmd_stream.str()]["ABW3"]["x"].as<float>(), abw_points[cmd_stream.str()]["ABW3"]["y"].as<float>()});
    points.push_back(
        {abw_points[cmd_stream.str()]["ABW4"]["x"].as<float>(), abw_points[cmd_stream.str()]["ABW4"]["y"].as<float>()});
    points.push_back(
        {abw_points[cmd_stream.str()]["ABW5"]["x"].as<float>(), abw_points[cmd_stream.str()]["ABW5"]["y"].as<float>()});
    points.push_back(
        {abw_points[cmd_stream.str()]["ABW6"]["x"].as<float>(), abw_points[cmd_stream.str()]["ABW6"]["y"].as<float>()});

    if (!previous_abws.empty()) {
      if (!CheckResult(points, previous_abws, diff)) {
        std::cout << abw_points[cmd_stream.str()]["image"].as<std::string>()
                  << " compared with previous image: " << previous_image << "\n\n";
      }
    }

    previous_abws  = points;
    previous_image = abw_points[cmd_stream.str()]["image"].as<std::string>();
  }
  return 0;
}
