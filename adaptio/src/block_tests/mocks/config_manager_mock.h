#pragma once

#include <boost/outcome/result.hpp>
#include <filesystem>
#include <optional>
#include <string>

#include "common/tolerances/tolerances_configuration.h"
#include "configuration/config_manager.h"
#include "controller/controller_configuration.h"
#include "main/calibration/calibration_configuration.h"
#include "main/image_logging/image_logging_configuration.h"
#include "main/joint_geometry/joint_geometry.h"
#include "main/weld_control/weld_control_types.h"
#include "scanner/image_provider/image_provider_configuration.h"
#include "scanner/scanner_calibration_configuration.h"
#include "scanner/scanner_configuration.h"

namespace configuration {

/**
 * @brief Mock implementation of ConfigManager for testing.
 *
 * Provides configurable return values for all configuration types.
 * Useful for block tests where specific configuration values are needed.
 */
class ConfigManagerMock : public ConfigManager {
 public:
  ConfigManagerMock()           = default;
  ~ConfigManagerMock() override = default;

  // ConfigManager interface implementation
  auto Init(const std::filesystem::path& default_config, std::optional<std::filesystem::path> cmd_line_config,
            const std::filesystem::path& path_data) -> boost::outcome_v2::result<void> override {
    init_called_ = true;
    return init_result_;
  }

  auto GetController() -> controller::ControllerConfigurationData override { return controller_config_; }

  auto GetImageProvider() -> scanner::image_provider::ImageProviderConfigData override {
    return image_provider_config_;
  }

  auto GetScannerCalibration(const std::string& scanner_serial_number)
      -> std::optional<scanner::ScannerCalibrationData> override {
    if (scanner_calibration_config_.has_value()) {
      return scanner_calibration_config_;
    }
    return std::nullopt;
  }

  auto GetScanner() -> scanner::ScannerConfigurationData override { return scanner_config_; }

  auto GetCalibrationFixtureJointGeometry() -> joint_geometry::JointGeometry override {
    return calibration_joint_geometry_config_;
  }

  auto GetWeldControlConfiguration() -> weld_control::Configuration override { return weld_control_config_; }

  auto GetTolerancesConfiguration() -> tolerances::Configuration override { return tolerances_config_; }

  auto GetCalibrationConfiguration() -> calibration::Configuration override { return calibration_config_; }

  auto GetImageLoggingConfiguration() -> image_logging::Configuration override { return image_logging_config_; }

  auto GetScannerConfig() -> scanner::ScannerConfigurationData { return GetScanner(); }
  auto GetImageProviderConfig() -> scanner::image_provider::ImageProviderConfigData { return GetImageProvider(); }

  // Mock configuration setters for testing
  void SetInitResult(boost::outcome_v2::result<void> result) { init_result_ = result; }

  void SetControllerConfig(const controller::ControllerConfigurationData& config) { controller_config_ = config; }

  void SetImageProviderConfig(const scanner::image_provider::ImageProviderConfigData& config) {
    image_provider_config_ = config;
  }

  void SetScannerCalibrationConfig(const scanner::ScannerCalibrationData& config) {
    scanner_calibration_config_ = config;
  }

  void SetScannerConfig(const scanner::ScannerConfigurationData& config) { scanner_config_ = config; }

  void SetCalibrationFixtureJointGeometry(const joint_geometry::JointGeometry& config) {
    calibration_joint_geometry_config_ = config;
  }

  void SetWeldControlConfiguration(const weld_control::Configuration& config) { weld_control_config_ = config; }

  void SetTolerancesConfiguration(const tolerances::Configuration& config) { tolerances_config_ = config; }

  void SetCalibrationConfiguration(const calibration::Configuration& config) { calibration_config_ = config; }

  void SetImageLoggingConfiguration(const image_logging::Configuration& config) { image_logging_config_ = config; }

  // Test helper methods
  auto WasInitCalled() const -> bool { return init_called_; }

  void Reset() {
    init_called_ = false;
    init_result_ = boost::outcome_v2::success();
    // Reset all configurations to default values
    controller_config_                 = {};
    image_provider_config_             = {};
    scanner_calibration_config_        = std::nullopt;
    scanner_config_                    = {};
    calibration_joint_geometry_config_ = {};
    weld_control_config_               = {};
    tolerances_config_                 = {};
    calibration_config_                = {};
    image_logging_config_              = {};
  }

 private:
  // Mock state
  bool init_called_                            = false;
  boost::outcome_v2::result<void> init_result_ = boost::outcome_v2::success();

  // Configuration data
  controller::ControllerConfigurationData controller_config_{};
  scanner::image_provider::ImageProviderConfigData image_provider_config_{};
  std::optional<scanner::ScannerCalibrationData> scanner_calibration_config_ = std::nullopt;
  scanner::ScannerConfigurationData scanner_config_{};
  joint_geometry::JointGeometry calibration_joint_geometry_config_{};
  weld_control::Configuration weld_control_config_{};
  tolerances::Configuration tolerances_config_{};
  calibration::Configuration calibration_config_{};
  image_logging::Configuration image_logging_config_{};
};

}  // namespace configuration
