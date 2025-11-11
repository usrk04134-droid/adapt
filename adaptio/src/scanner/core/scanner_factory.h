#pragma once

#include <prometheus/registry.h>

#include "scanner/core/scanner.h"
#include "scanner/core/scanner_calibration_configuration.h"
#include "scanner/core/scanner_configuration.h"
#include "scanner/image_logger/image_logger.h"
#include "scanner/image_provider/image_provider.h"
#include "scanner/image_provider/image_provider_configuration.h"
#include "scanner/joint_model/joint_model.h"

namespace scanner {

class ScannerFactory {
 public:
  virtual ~ScannerFactory() = default;

  virtual auto CreateImageProvider(const image_provider::ImageProviderConfigData& image_provider_config,
                                   prometheus::Registry* registry) -> image_provider::ImageProviderPtr        = 0;
  virtual auto CreateScanner(image_provider::ImageProvider* image_provider,
                             const ScannerCalibrationData& scanner_calibration,
                             const ScannerConfigurationData& scanner_configuration, const image_provider::Fov& fov,
                             const joint_model::JointProperties& joint_properties, ScannerOutputCB* scanner_output,
                             image_logger::ImageLogger* logger, prometheus::Registry* registry) -> ScannerPtr = 0;

  virtual auto CreateImageLogger() -> image_logger::ImageLoggerPtr = 0;
};

auto GetFactory() -> ScannerFactory*;

// For test
void SetFactoryGenerator(std::function<ScannerFactory*()> generator);
}  // namespace scanner
