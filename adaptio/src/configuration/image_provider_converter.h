#pragma once

#include <any>
#include <filesystem>
#include <string>
#include <unordered_map>

#include "common/data/data_value.h"
#include "common/file/yaml.h"
#include "common/logging/application_log.h"
#include "conf_file_handler.h"
#include "configuration/converter.h"
#include "configuration_error.h"
#include "scanner/image_provider/image_provider_configuration.h"

namespace configuration {

using common::data::DataValue;
using common::file::YamlPtr;

static auto IsAbsolutePath(const std::string& path) -> bool { return !path.empty() && path[0] == '/'; }

static auto CleanUpPath(const std::string& path) -> std::string {
  auto search_string  = std::string("/./");
  auto replace_string = std::string("/");
  auto result         = path;

  auto s_size       = search_string.size();
  std::size_t index = result.find(search_string);

  while (index != std::string::npos) {
    result.replace(index, s_size, replace_string);
    index = result.find(search_string);
  }

  return result;
}

class ImageProviderCoverter {
 public:
  static auto ToStruct(const std::unordered_map<std::string, DataValue>& map, const std::filesystem::path& yaml_file,
                       scanner::image_provider::ImageProviderConfigData& config) -> boost::outcome_v2::result<void> {
    const std::string tag = "configuration/image_provider";

    if (map.contains(tag + "/type")) {
      if (map.at(tag + "/type").Value<std::string>().value() == "basler") {
        config.image_provider = scanner::image_provider::ImageProviderType::BASLER;
      } else {
        config.image_provider = scanner::image_provider::ImageProviderType::SIMULATION;
      }
    }
    auto config_file_path = std::filesystem::absolute(yaml_file).parent_path().string();

    try {
      TryUpdate(map, tag + "/basler/gain", config.basler_config.gain);
      TryUpdate(map, tag + "/basler/exposure_time", config.basler_config.exposure_time);

      TryUpdate(map, tag + "/fov/width", config.fov.width);
      TryUpdate(map, tag + "/fov/height", config.fov.height);
      TryUpdate(map, tag + "/fov/offset_x", config.fov.offset_x);
      TryUpdate(map, tag + "/fov/offset_y", config.fov.offset_y);

      if (config.image_provider == scanner::image_provider::ImageProviderType::SIMULATION) {
        TryUpdate(map, tag + "/simulation/realtime", config.sim_config.realtime);
        if (map.contains(tag + "/simulation/images")) {
          auto image_path = map.at(tag + "/simulation/images").Value<std::string>().value();
          if (!IsAbsolutePath(image_path)) {
            image_path = config_file_path + "/" + image_path;
          }

          config.sim_config.images_path = CleanUpPath(image_path);
        }
      }
    } catch (const std::exception& e) {
      LOG_ERROR("Format of configuration file is not supported: file: {} error: {}", yaml_file.string(), e.what());
      return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_READ_FILE_ERROR));
    }
    return boost::outcome_v2::success();
  }
};
}  // namespace configuration
