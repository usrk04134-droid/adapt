#pragma once

#include <filesystem>
#include <string>

#include "common/data/data_value.h"
#include "common/file/yaml.h"
#include "common/logging/application_log.h"
#include "configuration_error.h"
#include "converter.h"
#include "scanner/core/scanner_configuration.h"

namespace configuration {

using common::data::DataValue;
using common::file::YamlPtr;

class ScannerConverter {
 public:
  static auto ToStruct(const std::unordered_map<std::string, DataValue>& map, const std::filesystem::path& yaml_file,
                       scanner::ScannerConfigurationData& config) -> boost::outcome_v2::result<void> {
    const std::string tag = "configuration/scanner";

    try {
      TryUpdate(map, tag + "/filtering/gray_minimum_top", config.gray_minimum_top);
      TryUpdate(map, tag + "/filtering/gray_minimum_wall", config.gray_minimum_wall);
      TryUpdate(map, tag + "/filtering/gray_minimum_bottom", config.gray_minimum_bottom);

    } catch (const std::exception& e) {
      LOG_ERROR("Format of configuration file is not supported: file: {} error: {}", yaml_file.string(), e.what());
      return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_READ_FILE_ERROR));
    }
    return boost::outcome_v2::success();
  }
};

}  // namespace configuration
