#pragma once

#include <filesystem>
#include <string>

#include "common/file/yaml.h"
#include "common/logging/application_log.h"
#include "configuration/converter.h"
#include "configuration_error.h"
#include "image_logging/image_logging_configuration.h"

namespace configuration {

using common::data::DataValue;

class ImageLoggingConfigurationConverter {
 public:
  static auto ToStruct(const std::unordered_map<std::string, DataValue>& map, const std::filesystem::path& yaml_file,
                       image_logging::Configuration& config) -> boost::outcome_v2::result<void> {
    const std::string tag = "configuration/image_logging";

    try {
      std::string mode;
      TryUpdate(map, tag + "/mode", mode);
      config.mode = image_logging::ModeFromString(mode).value_or(image_logging::Mode::OFF);
      TryUpdate(map, tag + "/sample_rate", config.sample_rate);
      TryUpdate(map, tag + "/buffer_size", config.buffer_size);
      TryUpdate(map, tag + "/on_error_interval_ms", config.on_error_interval);
      TryUpdate(map, tag + "/path", config.path);
    } catch (const std::exception& e) {
      LOG_ERROR("Format of configuration file is not supported: file: {} error: {}", yaml_file.string(), e.what());
      return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_READ_FILE_ERROR));
    }
    return boost::outcome_v2::success();
  }
};

}  // namespace configuration
