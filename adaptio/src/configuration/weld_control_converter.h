#pragma once

#include <filesystem>
#include <string>

#include "common/file/yaml.h"
#include "common/logging/application_log.h"
#include "configuration/converter.h"
#include "configuration_error.h"
#include "weld_control/weld_control_types.h"

namespace configuration {

using common::data::DataValue;

class WeldControlConfigurationConverter {
 public:
  static auto ToStruct(const std::unordered_map<std::string, DataValue>& map, const std::filesystem::path& yaml_file,
                       weld_control::Configuration& config) -> boost::outcome_v2::result<void> {
    const std::string tag = "configuration/weld_control";

    try {
      TryUpdate(map, tag + "/scanner_groove_geometry_update/tolerance/upper_width",
                config.scanner_groove_geometry_update.tolerance.upper_width);
      TryUpdate(map, tag + "/scanner_groove_geometry_update/tolerance/wall_angle",
                config.scanner_groove_geometry_update.tolerance.wall_angle);
      TryUpdate(map, tag + "/supervision/arcing_lost_grace_ms", config.supervision.arcing_lost_grace);
      TryUpdate(map, tag + "/scanner_input_interval_ms", config.scanner_input_interval);
      TryUpdate(map, tag + "/adaptivity/gaussian_filter/kernel_size", config.adaptivity.gaussian_filter.kernel_size);
      TryUpdate(map, tag + "/adaptivity/gaussian_filter/sigma", config.adaptivity.gaussian_filter.sigma);
      TryUpdate(map, tag + "/adaptivity/current_adaptivity_max_gain", config.adaptivity.current_adaptivity_max_gain);
      TryUpdate(map, tag + "/adaptivity/speed_adaptivity_max_gain", config.adaptivity.speed_adaptivity_max_gain);
      TryUpdate(map, tag + "/handover_grace_seconds", config.handover_grace);
      TryUpdate(map, tag + "/scanner_low_confidence_grace_seconds", config.scanner_low_confidence_grace);
      TryUpdate(map, tag + "/scanner_no_confidence_grace_seconds", config.scanner_no_confidence_grace);
      TryUpdate(map, tag + "/weld_data_persistent_storage", config.weld_data_persistent_storage);

    } catch (const std::exception& e) {
      LOG_ERROR("Format of configuration file is not supported: file: {} error: {}", yaml_file.string(), e.what());
      return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_READ_FILE_ERROR));
    }
    return boost::outcome_v2::success();
  }
};

}  // namespace configuration
