#pragma once

#include <any>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <regex>
#include <string>

#include "common/data/data_value.h"
#include "common/file/yaml.h"
#include "common/logging/application_log.h"
#include "configuration_error.h"
#include "controller/controller_configuration.h"
#include "converter.h"

namespace configuration {

using common::data::DataValue;
using common::file::YamlPtr;
using controller::ControllerConfigurationData;

inline auto HexOctetToByte(const char octet[2]) -> std::optional<uint8_t> {
  if (octet == nullptr) return std::nullopt;

  uint8_t value = 0;

  for (int i = 0; i < 2; i++) {
    char c = static_cast<char>(tolower(octet[i]));

    value <<= 4;

    if (c >= '0' && c <= '9') {
      value += (c - '0');
    } else if (c >= 'a' && c <= 'f') {
      value += (10 + c - 'a');
    } else {
      return std::nullopt;
    }
  }

  return value;
}

class ControllerConverter {
 public:
  static auto ToStruct(const std::unordered_map<std::string, DataValue>& map, const std::filesystem::path& yaml_file,
                       ControllerConfigurationData& config) -> boost::outcome_v2::result<void> {
    const std::string tag = "configuration/controller";
    try {
      TryUpdate(map, tag + "/cycle_time_ms", config.cycle_time_ms);

      std::string type;
      TryUpdate(map, tag + "/type", type);
      if (type == "pn_driver") {
        config.type = controller::ControllerType::PN_DRIVER;
      } else if (type == "simulation") {
        config.type = controller::ControllerType::SIMULATION;
      }

      if (config.type == controller::ControllerType::PN_DRIVER) {
        if (!config.pn_driver.has_value()) {
          config.pn_driver = controller::pn_driver::Configuration{};
        }
      }

      if (map.contains(tag + "/pn_driver/mac_address")) {
        if (!config.pn_driver.has_value()) {
          config.pn_driver = controller::pn_driver::Configuration{};
        }

        auto interface = map.at(tag + "/pn_driver/mac_address").Value<std::string>().value();
        std::regex mac_regex("^([0-9a-f]{2}):([0-9a-f]{2}):([0-9a-f]{2}):([0-9a-f]{2}):([0-9a-f]{2}):([0-9a-f]{2})$",
                             std::regex_constants::icase);
        std::smatch matches;

        if (regex_search(interface, matches, mac_regex) && matches.size() == 7) {
          for (int i = 1; i < 7; i++) {  // Start at 1 since 0 is the complete match
            config.pn_driver.value().mac_address.push_back(HexOctetToByte(matches[i].str().c_str()).value());
          }
        } else {
          LOG_ERROR("Interface MAC-address should be in the form 11:22:33:44:55:66");
        }
      }

      if (map.contains(tag + "/pn_driver/interface")) {
        if (!config.pn_driver.has_value()) {
          config.pn_driver = controller::pn_driver::Configuration{};
        }
      }
      TryUpdate(map, tag + "/pn_driver/interface", config.pn_driver.value().interface);

      if (config.pn_driver.has_value() && config.pn_driver.value().mac_address.empty() &&
          config.pn_driver.value().interface.empty()) {
        LOG_ERROR("At least one of mac_address and interface must be defined.");
        return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_READ_FILE_ERROR));
      }

      TryUpdate(map, tag + "/pn_driver/rema_path", config.pn_driver.value().rema_path);

      if (config.pn_driver.has_value() && config.pn_driver.value().rema_path.empty()) {
        LOG_ERROR("No REMA file path set for pn_driver");
        return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_READ_FILE_ERROR));
      }
    } catch (const std::exception& e) {
      LOG_ERROR("Format of configuration file is not supported: file: {} error: {}", yaml_file.string(), e.what());
      return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_READ_FILE_ERROR));
    }
    return boost::outcome_v2::success();
  }
};

}  // namespace configuration
