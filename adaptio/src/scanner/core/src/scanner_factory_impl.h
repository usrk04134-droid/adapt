#pragma once

#include <prometheus/registry.h>

#include "scanner/core/scanner.h"
#include "scanner/core/scanner_factory.h"
#include "scanner/image_logger/image_logger.h"
#include "scanner/image_provider/image_provider_configuration.h"

namespace scanner {

class ScannerFactoryImpl : public ScannerFactory {
 public:
  auto CreateImageProvider(const image_provider::ImageProviderConfigData& image_provider_config,
                           prometheus::Registry* registry) -> image_provider::ImageProviderPtr override;
  auto CreateScanner(image_provider::ImageProvider* image_provider, const ScannerCalibrationData& scanner_calibration,
                     const ScannerConfigurationData& scanner_configuration, const image_provider::Fov& fov,
                     const joint_model::JointProperties& joint_properties, ScannerOutputCB* scanner_output,
                     image_logger::ImageLogger* logger, prometheus::Registry* registry) -> ScannerPtr override;

  auto CreateImageLogger() -> image_logger::ImageLoggerPtr override;
};

}  // namespace scanner
