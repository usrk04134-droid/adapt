#include "config_manager_impl.h"

#include <any>
#include <boost/outcome/result.hpp>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "common/file/yaml.h"
#include "common/logging/application_log.h"
#include "conf_factory.h"
#include "configuration/calib_fix_joint_geometry_converter.h"
#include "configuration/calibration_converter.h"
#include "configuration/config_manager.h"
#include "configuration/configuration_error.h"
#include "configuration/controller_config_converter.h"
#include "configuration/converter.h"
#include "configuration/image_logging_converter.h"
#include "configuration/image_provider_converter.h"
#include "configuration/scanner_calibration_converter.h"
#include "configuration/scanner_converter.h"
#include "configuration/tolerances_converter.h"
#include "configuration/weld_control_converter.h"
#include "scanner/scanner_calibration_configuration.h"

using configuration::ConfigManagerImpl;

ConfigManagerImpl::ConfigManagerImpl(const fs::path& path_scanner_calibration)
    : path_scanner_calibration_(path_scanner_calibration) {
  auto* factory = GetFactory();
  fh_           = factory->CreateFileHandler();
}

auto ConfigManagerImpl::Init(const fs::path& default_config, std::optional<fs::path> cmd_line_config,
                             fs::path const& path_data) -> boost::outcome_v2::result<void> {
  // Get default configuration files. Must exist
  LOG_INFO("Default config: {}", default_config.string());
  LOG_INFO("User config path: {}", path_data.string());
  if (cmd_line_config.has_value()) {
    LOG_INFO("Cmd line config: {}", cmd_line_config.value().string());
  }
  // Read default config files (/etc/adaptio)
  auto result = ReadConfigurationFile(default_config, true);
  if (!result) {
    LOG_ERROR("Default config file not found: {}", default_config.string());
    return result;
  }

  TryCopyConfigFiles(default_config, path_data);

  // Read user config files if any (/var/lib/adaptio)
  std::filesystem::path const top_config(path_data / "configuration.yaml");
  LOG_INFO("User config: {}", top_config.string());

  if (fh_->FileExist(top_config)) {
    result = ReadConfigurationFile(top_config);
    if (!result) {
      LOG_ERROR("Could not get user config");
      return result;
    }
  }

  // Read user config file provided from cmd line
  if (cmd_line_config.has_value()) {
    const auto& file = cmd_line_config.value();
    LOG_INFO("User config from cmd line: {}", file.string());

    result = ReadConfigurationFile(file);
    if (!result) {
      LOG_ERROR("User config file from command line not found: {}", file.string());
      return result;
    }
  }

  result = CheckConfigFiles();
  if (!result) {
    return result;
  }

  // Read the configuration files (will later be moved to data base) refered in configuration.yaml
  return ReadConfigFiles();
}

auto ConfigManagerImpl::CheckConfigFiles() -> boost::outcome_v2::result<void> {
  LOG_DEBUG("CheckConfigFiles");
  for (const auto& config : converters_) {
    auto tag  = config.first;
    auto file = config.second.first;

    LOG_INFO("Configuration file used: tag: {} file: {}", tag, file.string());

    if (file.empty()) {
      LOG_ERROR("Config file for tag not set: {}", file.string());
      return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_READ_FILE_ERROR));
    }

    if (!fh_->FileExist(file)) {
      LOG_ERROR("File for configuration does not exist: tag : {} file: {}", tag, file.string());
      return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_WRITE_FILE_MISSING));
    }
  }

  return boost::outcome_v2::success();
}

auto ConfigManagerImpl::ReadConfigFiles() -> boost::outcome_v2::result<void> {
  auto* factory = GetFactory();

  for (auto& config : converters_) {
    auto tag  = config.first;
    auto file = config.second.first;

    auto converter = factory->CreateConverter(tag, file);

    if (converter) {
      auto result = converter->ReadPersistentData();
      if (!result) {
        LOG_ERROR("Failed to read configuration file with tag: {}", config.first);
        return result;
      }
    }
    converters_[tag].second = std::move(converter);
  }

  return boost::outcome_v2::success();
}

auto ConfigManagerImpl::ReadConfigurationFile(const fs::path& config_file, bool must_exist)
    -> boost::outcome_v2::result<void> {
  LOG_DEBUG("Configuration: Parse config file on top level: {}", config_file.string());

  if (!fh_->FileExist(config_file.string())) {
    if (must_exist) {
      return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_WRITE_FILE_MISSING));
    }
    return boost::outcome_v2::success();
  }

  auto yaml_string              = fh_->ReadFile(config_file.string());
  auto maybe_configuration_yaml = common::file::Yaml::FromString(yaml_string, TAG_CONF);
  if (maybe_configuration_yaml.has_error()) {
    LOG_ERROR("Could not parse yaml string with error code: {}", maybe_configuration_yaml.error().to_string());
    return boost::outcome_v2::failure(make_error_code(ConfigurationErrorCode::CONFIGURATION_READ_FILE_ERROR));
  }

  auto config_map = maybe_configuration_yaml.value()->AsUnorderedMap();

  auto parent_path = fh_->GetAbsParent(config_file);

  for (auto& config : config_map) {
    // Key is "configuration/<tag>", extract tag
    auto key       = config.first;
    const auto tag = key.substr(key.find('/') + 1, key.length() - 1);

    if (!converters_.contains(tag)) {
      LOG_TRACE("Configuration tag does not exist: {}", tag);
      continue;
    }

    const auto file_path = config.second.Value<std::string>().value();
    auto sub_config_file = parent_path + "/" + file_path;
    if (!file_path.empty() && file_path[0] == '/') {
      // Absolute path
      sub_config_file = file_path;
    }
    converters_[tag].first = sub_config_file;
  }

  auto result = ControllerConverter::ToStruct(config_map, config_file, controller_config_);
  if (result) {
    result = ScannerConverter::ToStruct(config_map, config_file, scanner_config_);
  }
  if (result) {
    result = ImageProviderCoverter::ToStruct(config_map, config_file, image_provider_config_);
  }
  if (result) {
    result = CalibFixJointGeometryConverter::ToStruct(config_map, config_file, calibration_joint_geometry_config_);
  }
  if (result) {
    result = WeldControlConfigurationConverter::ToStruct(config_map, config_file, weld_control_config_);
  }
  if (result) {
    result = TolerancesConverter::ToStruct(config_map, config_file, tolerances_config_);
  }
  if (result) {
    result = CalibrationConfigurationConverter::ToStruct(config_map, config_file, calibration_config_);
  }
  if (result) {
    result = ImageLoggingConfigurationConverter::ToStruct(config_map, config_file, image_logging_config_);
  }
  if (!result) {
    return result;
  }

  return boost::outcome_v2::success();
}

void ConfigManagerImpl::TryCopyConfigFiles(fs::path const& default_config, fs::path const& path_data) {
  // Try copy the files refered in the configuration.yaml from /etc/adaptio to /var/lib/adaptio
  // If they not already exist in /var/lib/adaptio
  auto try_copy_file = [this, default_config, path_data](auto const& sub_config) {
    auto from_file = fs::path(default_config.parent_path() / sub_config.first.filename());
    auto to_file   = path_data.string() + "/" + from_file.filename().string();
    if (fs::exists(from_file) && !fs::exists(to_file)) {
      fs::copy_file(from_file, to_file, fs::copy_options::overwrite_existing);
      fh_->SetWritePermission(to_file);
    }
  };
}

auto ConfigManagerImpl::GetScannerCalibration(const std::string& scanner_serial_number)
    -> std::optional<scanner::ScannerCalibrationData> {
  auto const path_directory = std::filesystem::path(path_scanner_calibration_);

  if (!fs::exists(path_directory) && !fs::is_directory(path_directory)) {
    LOG_ERROR("Invalid scanner-configuration path: {}", path_directory.string());
    return {};
  }

  LOG_DEBUG("Scanning for scanner-calibration files in: {}", path_directory.string());

  try {
    for (const auto& entry : fs::directory_iterator(path_directory)) {
      auto const path      = entry.path().string();
      auto const extension = entry.path().extension();
      if (extension != ".yaml" && extension != ".yml") {
        LOG_DEBUG("Skipping non-yaml file: {}", path);
        continue;
      }

      LOG_DEBUG("Reading file: {}", path);

      ScannerCalibrationConverter converter(TAG_SC, path);
      auto const res = converter.ReadPersistentData();
      if (res.has_failure()) {
        LOG_ERROR("Failed to read file with error: {}", res.error().message());
        continue;
      }

      auto config = std::any_cast<scanner::ScannerCalibrationData>(converter.GetConfig());

      if (scanner_serial_number.empty() || scanner_serial_number == config.scanner_serial_number) {
        LOG_INFO("Found scanner calibration file for serial-number {} at {}", config.scanner_serial_number, path);
        return config;
      }
    }
  } catch (const fs::filesystem_error& ex) {
    LOG_ERROR("Filesystem error during directory iteration: {}", ex.what());
  }

  LOG_ERROR("Unable to find scanner-calibration file for serial-number: {} in directory: {}", scanner_serial_number,
            path_directory.string());

  return {};
}
