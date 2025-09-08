#pragma once

#include <filesystem>
#include <string>

#include "common/file/yaml.h"
#include "common/logging/application_log.h"
#include "configuration/converter.h"
#include "configuration_error.h"
#include "main/calibration/calibration_configuration.h"

namespace configuration {

using common::data::DataValue;

class CalibrationConfigurationConverter {
 public:
  static auto ToStruct(const std::unordered_map<std::string, DataValue>& map, const std::filesystem::path& yaml_file,
                       calibration::Configuration& config) -> boost::outcome_v2::result<void> {
    const std::string tag = "configuration/calibration";

    try {
      // grid_config
      TryUpdate(map, tag + "/grid_config/margin_top", config.grid_config.margin_top);
      TryUpdate(map, tag + "/grid_config/margin_x", config.grid_config.margin_x);
      TryUpdate(map, tag + "/grid_config/margin_z", config.grid_config.margin_z);
      TryUpdate(map, tag + "/grid_config/margin_c", config.grid_config.margin_c);
      TryUpdate(map, tag + "/grid_config/target_nr_gridpoints", config.grid_config.target_nr_gridpoints);

      // runner_config
      TryUpdate(map, tag + "/runner_config/slide_velocity", config.runner_config.slide_velocity);
      TryUpdate(map, tag + "/runner_config/stabilization_time", config.runner_config.stabilization_time);
      TryUpdate(map, tag + "/runner_config/near_target_delta", config.runner_config.near_target_delta);
      TryUpdate(map, tag + "/runner_config/max_time_per_observation", config.runner_config.max_time_per_observation);

    } catch (const std::exception& e) {
      LOG_ERROR("Format of configuration file is not supported: file: {} error: {}", yaml_file.string(), e.what());
      return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_READ_FILE_ERROR));
    }
    return boost::outcome_v2::success();
  }
};

}  // namespace configuration
