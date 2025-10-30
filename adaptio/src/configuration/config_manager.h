#pragma once

#include <boost/outcome/result.hpp>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>

#include "common/tolerances/tolerances_configuration.h"
#include "controller/controller_configuration.h"
#include "main/calibration/calibration_configuration.h"
#include "main/image_logging/image_logging_configuration.h"
#include "main/joint_geometry/joint_geometry.h"
#include "main/weld_control/weld_control_types.h"
#include "scanner/core/scanner_calibration_configuration.h"
#include "scanner/core/scanner_configuration.h"
#include "scanner/image_provider/image_provider_configuration.h"

namespace configuration {

const std::string TAG_CONF = "configuration";
const std::string TAG_SC   = "scanner_calibration";

class ConfigurationHandle;

/**
 * @brief Interface for configuration management.
 *
 * Provides access to various configuration data types used throughout the application.
 * Implementations handle loading, parsing, and providing access to configuration files.
 */
class ConfigManager {
 public:
  virtual ~ConfigManager() = default;

  /**
   * @brief Initialize the configuration manager with config file paths.
   *
   * @param default_config Path to default configuration files
   * @param cmd_line_config Optional command line specified config path
   * @param path_data Path to data directory for user configs
   * @return Result indicating success or failure
   */
  virtual auto Init(const std::filesystem::path& default_config, std::optional<std::filesystem::path> cmd_line_config,
                    const std::filesystem::path& path_data) -> boost::outcome_v2::result<void> = 0;

  // Configuration getters
  virtual auto GetController() -> controller::ControllerConfigurationData             = 0;
  virtual auto GetImageProvider() -> scanner::image_provider::ImageProviderConfigData = 0;
  virtual auto GetScannerCalibration(const std::string& scanner_serial_number)
      -> std::optional<scanner::ScannerCalibrationData>                              = 0;
  virtual auto GetScanner() -> scanner::ScannerConfigurationData                     = 0;
  virtual auto GetCalibrationFixtureJointGeometry() -> joint_geometry::JointGeometry = 0;
  virtual auto GetWeldControlConfiguration() -> weld_control::Configuration          = 0;
  virtual auto GetTolerancesConfiguration() -> tolerances::Configuration             = 0;
  virtual auto GetCalibrationConfiguration() -> calibration::Configuration           = 0;
  virtual auto GetImageLoggingConfiguration() -> image_logging::Configuration        = 0;
};

}  // namespace configuration
