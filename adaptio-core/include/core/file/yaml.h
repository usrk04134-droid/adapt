#pragma once

#include <yaml-cpp/yaml.h>

#include <boost/outcome.hpp>
#include <filesystem>
#include <unordered_map>

#include "core/data/data_value.h"

namespace core::file {

enum class YamlErrorCode : uint32_t {
  NO_ERROR        = 0,
  FILE_NOT_FOUND  = 1,
  DATA_EMPTY      = 2,
  FAILED_TO_PARSE = 3,
};

// NOLINTNEXTLINE(*-identifier-naming)
[[maybe_unused]] auto make_error_code(YamlErrorCode) -> std::error_code;

class Yaml;

using YamlPtr = std::unique_ptr<Yaml>;

class Yaml {
 public:
  Yaml(Yaml&)                     = delete;
  auto operator=(Yaml&) -> Yaml&  = delete;
  Yaml(Yaml&&)                    = delete;
  auto operator=(Yaml&&) -> Yaml& = delete;

  ~Yaml() = default;

  /**
   *
   * @param path The path to a yaml file
   * @return A unique_ptr to a Yaml object
   */
  static auto FromFile(const std::filesystem::path& path, const std::string& tag) -> boost::outcome_v2::result<YamlPtr>;

  static auto FromString(const std::string& str, const std::string& tag) -> boost::outcome_v2::result<YamlPtr>;

  auto AsUnorderedMap() -> std::unordered_map<std::string, core::data::DataValue>;

  auto ToString(const std::unordered_map<std::string, core::data::DataValue>& parameters)
      -> boost::outcome_v2::result<std::string>;

 private:
  std::unordered_map<std::string, core::data::DataValue> data_values_;

  Yaml() = default;
  YAML::Node node_;

  static auto Parse(const std::string& breadcrumbs, const YAML::Node& node)
      -> std::vector<std::pair<std::string, core::data::DataValue>>;
};

}  // namespace core::file

namespace std {
template <>
struct is_error_code_enum<core::file::YamlErrorCode> : true_type {};
}  // namespace std
