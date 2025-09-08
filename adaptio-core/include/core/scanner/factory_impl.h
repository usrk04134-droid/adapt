#pragma once

#include <prometheus/registry.h>

#include "core/scanner/image_provider_configuration.h"
#include "factory.h"

namespace core::scanner {

class FactoryImpl : public Factory {
 public:
  auto CreateImageProvider(const ImageProviderConfigData& image_provider_config, prometheus::Registry* registry)
      -> ImageProviderPtr override;
  auto CreateScanner(ImageProvider* image_provider, const ScannerCalibrationData& scanner_calibration,
                     const ScannerConfigurationData& scanner_configuration, const Fov& fov,
                     const core::scanner::JointProperties& joint_properties, ScannerOutputCB* scanner_output,
                     ImageLogger* logger, prometheus::Registry* registry) -> ScannerPtr override;

  auto CreateImageLogger() -> ImageLoggerPtr override;
};

}  // namespace core::scanner
