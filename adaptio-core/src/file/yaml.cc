#include "core/file/yaml.h"

#include <fmt/core.h>
#include <yaml-cpp/node/detail/iterator_fwd.h>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <iostream>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

#include "core/data/data_value.h"
#include "core/logging/application_log.h"

using core::file::Yaml;
using core::file::YamlErrorCode;

namespace {

auto ToStringWithPrecision(double value, const int n = 15) -> std::string {
  std::ostringstream out;
  out << std::scientific << std::setprecision(n) << value;
  return std::move(out).str();
}

auto Split(const std::string& str, char separator) -> std::vector<std::string> {
  std::vector<std::string> strings;
  int start_index = 0;
  int end_index   = 0;
  for (int i = 0; i <= str.size(); i++) {
    // If we reached the end of the word or the end of the input.
    if (str[i] == separator || i == str.size()) {
      end_index = i;
      std::string temp;
      temp.append(str, start_index, end_index - start_index);
      strings.push_back(temp);
      start_index = end_index + 1;
    }
  }
  return strings;
}

struct ErrorCategory : std::error_category {
  auto name() const noexcept -> const char* final;          // NOLINT(*-use-nodiscard)
  auto message(int error_code) const -> std::string final;  // NOLINT(*-use-nodiscard)
  auto default_error_condition(int other) const noexcept    // NOLINT(*-use-nodiscard)
      -> std::error_condition final;                        // NOLINT(*-use-nodiscard)
};

auto ErrorCategory::name() const noexcept -> const char* { return "YamlError"; }

auto ErrorCategory::message(int error_code) const -> std::string {
  switch (static_cast<YamlErrorCode>(error_code)) {
    case YamlErrorCode::NO_ERROR:
      return "No error";
    case YamlErrorCode::FILE_NOT_FOUND:
      return "File not found";
    case YamlErrorCode::DATA_EMPTY:
      return "No data";
    case YamlErrorCode::FAILED_TO_PARSE:
      return "Failed to parse";
  }
}

auto ErrorCategory::default_error_condition(int other) const noexcept -> std::error_condition {
  switch (static_cast<YamlErrorCode>(other)) {
    case YamlErrorCode::FILE_NOT_FOUND:
      return std::errc::invalid_argument;
    default:
      return {other, *this};
  }
}

const ErrorCategory ERROR_CATEGORY{};

}  // namespace

[[maybe_unused]] auto core::file::make_error_code(YamlErrorCode error_code)
    -> std::error_code {  // NOLINT(*-identifier-naming)
  return {static_cast<int>(error_code), ERROR_CATEGORY};
}

auto Yaml::FromFile(const std::filesystem::path& path, const std::string& tag) -> boost::outcome_v2::result<YamlPtr> {
  if (!exists(path)) {
    return YamlErrorCode::FILE_NOT_FOUND;
  }

  auto yaml = std::unique_ptr<Yaml>(new Yaml());

  std::ifstream file_stream(path.c_str());
  std::string const yaml_string(std::istreambuf_iterator<char>{file_stream}, {});

  return FromString(yaml_string, tag);
}

auto Yaml::ToString(const std::unordered_map<std::string, core::data::DataValue>& parameters)
    -> boost::outcome_v2::result<std::string> {
  // FromString must have been called before this method. node_ is expected to have Nodes
  if (node_.size() == 0) {
    return YamlErrorCode::DATA_EMPTY;
  }

  for (const auto& parameter : parameters) {
    auto keys = Split(parameter.first, '/');
    // First should be tag, remove it
    keys.erase(keys.begin());

    YAML::iterator node_it = node_.begin();

    for (auto& key : keys) {
      while (node_it->first.as<std::string>() != key && node_it != node_it->end()) {
        node_it++;
      }

      if (node_it->first.as<std::string>() != key) {
        LOG_ERROR("Should not happen. Key should be available. Key: {}", key);
        return YamlErrorCode::FAILED_TO_PARSE;
      }

      if (key == keys.back()) {
        // Set data
        if (node_it->second.Tag() == "!matrix") {
          auto columns = node_it->second["columns"].as<size_t>();
          auto rows    = node_it->second["rows"].as<size_t>();

          for (size_t i = 0; i < rows * columns; i++) {
            auto matrix                = parameter.second.Value<data::Matrix>().value();
            node_it->second["data"][i] = ToStringWithPrecision(matrix.data.get()[i]);
          }

        } else if (node_it->second.IsScalar()) {
          node_it->second = parameter.second.ToValueString();
        } else {
          // Should not happen
          return YamlErrorCode::FAILED_TO_PARSE;
        }
      } else {
        node_it = node_it->second.begin();
      }
    }
  }

  std::stringstream stream;
  stream << node_;
  return stream.str();
}

auto Yaml::FromString(const std::string& str, const std::string& tag) -> boost::outcome_v2::result<YamlPtr> {
  auto yaml   = std::unique_ptr<Yaml>(new Yaml());
  yaml->node_ = YAML::Load(str);

  auto parameters = Parse(tag, yaml->node_);

  for (auto& parameter : parameters) {
    yaml->data_values_.insert_or_assign(parameter.first, parameter.second);
  }

  return yaml;
}

auto Yaml::AsUnorderedMap() -> std::unordered_map<std::string, core::data::DataValue> { return data_values_; }

// NOLINTNEXTLINE(*-no-recursion)
auto Yaml::Parse(const std::string& breadcrumbs, const YAML::Node& node)
    -> std::vector<std::pair<std::string, core::data::DataValue>> {
  std::vector<std::pair<std::string, core::data::DataValue>> parameters;

  if (node.IsMap()) {
    for (YAML::const_iterator it = node.begin(); it != node.end(); it++) {
      auto key              = it->first.as<std::string>();
      auto next_breadcrumbs = fmt::format("{}/{}", breadcrumbs, key);
      LOG_TRACE("{}", next_breadcrumbs);

      if (it->second.IsDefined()) {
        if (it->second.Tag() == "!matrix") {  // Parse as matrix
          auto matrix_node = it->second;

          if (!matrix_node["columns"] || !matrix_node["columns"].IsScalar()) {
            LOG_ERROR("Missing/malformed columns key, giving up");
          }

          if (!matrix_node["rows"] || !matrix_node["rows"].IsScalar()) {
            LOG_ERROR("Missing/malformed rows key, giving up");
          }

          if (!matrix_node["data"] || !matrix_node["data"].IsSequence()) {
            LOG_ERROR("Missing/malformed data key, giving up");
          }

          auto columns = matrix_node["columns"].as<size_t>();

          auto rows = matrix_node["rows"].as<size_t>();

          auto data   = std::make_shared<double[]>(rows * columns);
          auto matrix = core::data::Matrix{rows, columns, std::move(data)};

          for (size_t i = 0; i < rows * columns; i++) {
            matrix.data.get()[i] = matrix_node["data"][i].as<double>();
          }

          auto value = core::data::DataValue(matrix);

          LOG_TRACE("Adding parameter {} =\n{}", next_breadcrumbs, value.ToValueString());

          parameters.emplace_back(next_breadcrumbs, value);

        } else if (it->second.Tag() != "?") {  // Unknown tag
          LOG_WARNING(R"(Unknown tag: "{}", trying to parse as normal)", it->second.Tag());
        } else if (it->second.IsScalar()) {  // Parse as key+value
          auto maybe_value = core::data::DataValue::FromString(it->second.as<std::string>());

          if (!maybe_value.has_value()) {
            return parameters;
          }

          auto value = maybe_value.value();

          LOG_TRACE("Adding parameter {} = {}", next_breadcrumbs, value.ToValueString());

          parameters.emplace_back(next_breadcrumbs, value);
        } else if (it->second.IsMap()) {  // Continue parsing children
          auto child_parameters = Parse(next_breadcrumbs, it->second);
          parameters.insert(parameters.end(), child_parameters.begin(), child_parameters.end());
        } else {
          // TODO: ???
        }
      }
    }
  }

  return parameters;
}

#ifndef DOCTEST_CONFIG_DISABLE
// NOLINTBEGIN(*-magic-numbers)
#include <doctest/doctest.h>

TEST_SUITE("YAML parsing") {
  TEST_CASE("Empty document") {
    std::string yaml = "---";

    auto y          = Yaml::FromString(yaml, "adaptio").value();
    auto parameters = y->AsUnorderedMap();

    CHECK_EQ(parameters.size(), 0);
  }

  TEST_CASE("Simple document") {
    std::string yaml = R"(pi: 3.141500000000000e+00
file_path: /path)";

    auto y          = Yaml::FromString(yaml, "adaptio").value();
    auto parameters = y->AsUnorderedMap();

    CHECK_EQ(parameters.size(), 2);
    CHECK(parameters.find("adaptio/pi") != parameters.end());
    CHECK_LT(parameters.at("adaptio/pi").Value<double>().value() - 3.1415, 0.0001);
    CHECK_EQ(parameters.at("adaptio/file_path").Value<std::string>(), "/path");

    CHECK_EQ(yaml, y->ToString(parameters).value());
  }

  TEST_CASE("Nested document") {
    std::string yaml = R"(
---

constants:
  pi: 3.1415
)";

    std::string yaml_out = R"(constants:
  pi: 2.000000000000000e+00)";

    auto y          = Yaml::FromString(yaml, "adaptio").value();
    auto parameters = y->AsUnorderedMap();

    CHECK_EQ(parameters.size(), 1);
    CHECK(parameters.find("adaptio/constants/pi") != parameters.end());
    CHECK_LT(parameters.at("adaptio/constants/pi").Value<double>().value() - 3.1415, 0.0001);

    // Modify data
    parameters.at("adaptio/constants/pi") = core::data::DataValue(2.0);

    CHECK_EQ(yaml_out, y->ToString(parameters).value());
  }

  TEST_CASE("Sequence (array) document") {
    std::string yaml = R"(
---

R: !matrix
  columns: 3
  rows: 3
  data:
    [ 1.1, 2.0, 4.0,
      1.0, 2.0, 4.0,
      1.0, 2.0, 4.0 ]
)";

    std::string out_yaml = R"(R: !<!matrix>
  columns: 3
  rows: 3
  data: [1.100000000000000e+00, 2.000000000000000e+00, 4.000000000000000e+00, 1.000000000000000e+00, 3.000000000000000e+00, 4.000000000000000e+00, 1.000000000000000e+00, 2.000000000000000e+00, 4.000000000000000e+00])";
    auto y               = Yaml::FromString(yaml, "adaptio").value();
    auto parameters      = y->AsUnorderedMap();

    CHECK_EQ(parameters.size(), 1);
    CHECK(parameters.find("adaptio/R") != parameters.end());
    CHECK(parameters.at("adaptio/R").Value<core::data::Matrix>().has_value());

    // Modify matrix
    parameters.at("adaptio/R").Value<core::data::Matrix>()->data.get()[4] = 3.0;
    auto yaml_string                                                      = y->ToString(parameters).value();
    CHECK_EQ(yaml_string, out_yaml);

    // From/To once more
    y           = Yaml::FromString(yaml_string, "adaptio").value();
    yaml_string = y->ToString(y->AsUnorderedMap()).value();
    CHECK_EQ(yaml_string, out_yaml);
  }

  TEST_CASE("Example configuration") {
    std::string yaml = R"(
---

# The camera parameters are set during calibration
camera_parameters:
  # The intrinsic camera parameters are represented as a matrix:
  # [
  #   fx, 0,  cx,
  #    0, fy, cy,
  #    0,  0,  1,
  # ]
  #
  # Where:
  #   fx = focal length in x
  #   fy = focal length in y (fy = fx * a, a = aspect ratio)
  #   cx = optical center in x
  #   cy = optical center in y
  intrinsic: !matrix
    rows: 3
    columns: 3
    data: [ 1.0, 0.0, 0.5,
            0.0, 1.0, 0.5,
            0.0, 0.0, 1.0 ]
  fov:
    width: 3500
    height: 2500
    offset_x: 298
    offset_y: 0

  extrinsic:
    # Rotation matrix
    R: !matrix
      rows: 3
      columns: 3
      data: [ 9.999974673412257431e-01, 2.039705193809659024e-03,  9.512696023625968975e-04,
              0.000000000000000000e+00, 4.226691551490259768e-01, -9.062840533108859065e-01,
              -2.250624609754632317e-03, 9.062817580026263364e-01,  4.226680846722816187e-01 ]

    # Translation vector
    t: !matrix
      rows: 3
      columns: 1
      data: [ 0.000000000000000000e+00, 0.000000000000000000e+00, 4.087606157143235386e-01 ]

    # Distortion coefficients
    D: !matrix
      rows: 5
      columns: 1
      data: [ 0.1, 0.01, -0.001, 0, 0 ]

image_processing:
  # Initial guesses
  left_joint_angle: 0.1396263401595 # 8.0 / 360.0 * 2 * PI
  right_joint_angle: 0.1396263401595 # 8.0 / 360.0 * 2 * PI
  left_groove_depth: 0.0
  right_groove_depth: 0.0
)";

    auto y          = Yaml::FromString(yaml, "adaptio").value();
    auto parameters = y->AsUnorderedMap();

    CHECK(parameters.find("adaptio/camera_parameters/intrinsic") != parameters.end());
    CHECK(parameters.find("adaptio/camera_parameters/fov/width") != parameters.end());
    CHECK(parameters.find("adaptio/camera_parameters/fov/height") != parameters.end());
    CHECK(parameters.find("adaptio/camera_parameters/fov/offset_x") != parameters.end());
    CHECK(parameters.find("adaptio/camera_parameters/fov/offset_y") != parameters.end());
    CHECK(parameters.find("adaptio/camera_parameters/extrinsic/R") != parameters.end());
    CHECK(parameters.find("adaptio/camera_parameters/extrinsic/t") != parameters.end());
    CHECK(parameters.find("adaptio/camera_parameters/extrinsic/D") != parameters.end());
    CHECK(parameters.find("adaptio/image_processing/left_joint_angle") != parameters.end());
    CHECK(parameters.find("adaptio/image_processing/right_joint_angle") != parameters.end());
    CHECK(parameters.find("adaptio/image_processing/left_groove_depth") != parameters.end());
    CHECK(parameters.find("adaptio/image_processing/right_groove_depth") != parameters.end());

    // To/From
    auto yaml_string = y->ToString(parameters).value();

    y               = Yaml::FromString(yaml_string, "adaptio").value();
    auto new_params = y->AsUnorderedMap();

    CHECK(new_params.find("adaptio/camera_parameters/intrinsic") != parameters.end());
    CHECK(new_params.find("adaptio/camera_parameters/fov/width") != parameters.end());
    CHECK(new_params.find("adaptio/camera_parameters/fov/height") != parameters.end());
    CHECK(new_params.find("adaptio/camera_parameters/fov/offset_x") != parameters.end());
    CHECK(new_params.find("adaptio/camera_parameters/fov/offset_y") != parameters.end());
    CHECK(new_params.find("adaptio/camera_parameters/extrinsic/R") != parameters.end());
    CHECK(new_params.find("adaptio/camera_parameters/extrinsic/t") != parameters.end());
    CHECK(new_params.find("adaptio/camera_parameters/extrinsic/D") != parameters.end());
    CHECK(new_params.find("adaptio/image_processing/left_joint_angle") != parameters.end());
    CHECK(new_params.find("adaptio/image_processing/right_joint_angle") != parameters.end());
    CHECK(new_params.find("adaptio/image_processing/left_groove_depth") != parameters.end());
    CHECK(new_params.find("adaptio/image_processing/right_groove_depth") != parameters.end());
  }
}
// NOLINTEND(*-magic-numbers)

#endif
