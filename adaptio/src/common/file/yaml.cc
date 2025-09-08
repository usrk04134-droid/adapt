#include "common/file/yaml.h"

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

#include "common/data/data_value.h"
#include "common/logging/application_log.h"

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
  switch (static_cast<common::file::YamlErrorCode>(error_code)) {
    case common::file::YamlErrorCode::NO_ERROR:
      return "No error";
    case common::file::YamlErrorCode::FILE_NOT_FOUND:
      return "File not found";
    case common::file::YamlErrorCode::DATA_EMPTY:
      return "No data";
    case common::file::YamlErrorCode::FAILED_TO_PARSE:
      return "Failed to parse";
  }
}

auto ErrorCategory::default_error_condition(int other) const noexcept -> std::error_condition {
  switch (static_cast<common::file::YamlErrorCode>(other)) {
    case common::file::YamlErrorCode::FILE_NOT_FOUND:
      return std::errc::invalid_argument;
    default:
      return {other, *this};
  }
}

const ErrorCategory ERROR_CATEGORY{};

}  // namespace

[[maybe_unused]] auto common::file::make_error_code(YamlErrorCode error_code)
    -> std::error_code {  // NOLINT(*-identifier-naming)
  return {static_cast<int>(error_code), ERROR_CATEGORY};
}

namespace common::file {

auto Yaml::FromFile(const std::filesystem::path& path, const std::string& tag) -> boost::outcome_v2::result<YamlPtr> {
  if (!exists(path)) {
    return YamlErrorCode::FILE_NOT_FOUND;
  }

  auto yaml = std::unique_ptr<Yaml>(new Yaml());

  std::ifstream file_stream(path.c_str());
  std::string const yaml_string(std::istreambuf_iterator<char>{file_stream}, {});

  return FromString(yaml_string, tag);
}

auto Yaml::ToString(const std::unordered_map<std::string, common::data::DataValue>& parameters)
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

auto Yaml::AsUnorderedMap() -> std::unordered_map<std::string, common::data::DataValue> { return data_values_; }

// NOLINTNEXTLINE(*-no-recursion)
auto Yaml::Parse(const std::string& breadcrumbs, const YAML::Node& node)
    -> std::vector<std::pair<std::string, common::data::DataValue>> {
  std::vector<std::pair<std::string, common::data::DataValue>> parameters;

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
          auto matrix = common::data::Matrix{rows, columns, std::move(data)};

          for (size_t i = 0; i < rows * columns; i++) {
            matrix.data.get()[i] = matrix_node["data"][i].as<double>();
          }

          auto value = common::data::DataValue(matrix);

          LOG_TRACE("Adding parameter {} =\n{}", next_breadcrumbs, value.ToValueString());

          parameters.emplace_back(next_breadcrumbs, value);

        } else if (it->second.Tag() != "?") {  // Unknown tag
          LOG_WARNING(R"(Unknown tag: "{}", trying to parse as normal)", it->second.Tag());
        } else if (it->second.IsScalar()) {  // Parse as key+value
          auto maybe_value = common::data::DataValue::FromString(it->second.as<std::string>());

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
}  // namespace common::file
