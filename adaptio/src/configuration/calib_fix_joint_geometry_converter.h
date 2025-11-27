#pragma once

#include <any>
#include <filesystem>
#include <string>

#include "common/file/yaml.h"
#include "common/logging/application_log.h"
#include "conf_file_handler.h"
#include "configuration/converter.h"
#include "configuration_error.h"
#include "converter.h"
#include "joint_geometry/joint_geometry.h"

namespace configuration {

using common::data::DataValue;

class CalibFixJointGeometryConverter {
 public:
  static auto ToStruct(const std::unordered_map<std::string, DataValue>& map, const std::filesystem::path& yaml_file,
                       joint_geometry::JointGeometry& config) -> boost::outcome_v2::result<void> {
    const std::string tag = "configuration/calibration_fixture_joint_geometry";

    try {
      TryUpdate(map, tag + "/upper_joint_width", config.upper_joint_width_mm);
      TryUpdate(map, tag + "/groove_depth", config.groove_depth_mm);
      TryUpdate(map, tag + "/left_joint_angle", config.left_joint_angle_rad);
      TryUpdate(map, tag + "/right_joint_angle", config.right_joint_angle_rad);
      TryUpdate(map, tag + "/left_max_surface_angle", config.left_max_surface_angle_rad);
      TryUpdate(map, tag + "/right_max_surface_angle", config.right_max_surface_angle_rad);
      
      // Default to LONGITUDINAL if not specified in config
      config.welding_type = joint_geometry::WeldingType::LONGITUDINAL;
      std::string welding_type_str;
      if (TryUpdate(map, tag + "/welding_type", welding_type_str)) {
        config.welding_type = joint_geometry::WeldingTypeFromString(welding_type_str);
      }
    } catch (const std::exception& e) {
      LOG_ERROR("Format of configuration file is not supported: file: {} error: {}", yaml_file.string(), e.what());
      return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_READ_FILE_ERROR));
    }

    return boost::outcome_v2::success();
  }
};
}  // namespace configuration
