#pragma once

#include <prometheus/registry.h>

#include "core/scanner/image_logger.h"
#include "core/scanner/joint_model.h"
#include "image_provider.h"
#include "image_provider_configuration.h"
#include "scanner.h"
#include "scanner_calibration_configuration.h"
#include "scanner_configuration.h"
namespace core::scanner {

enum class ImageLoggerType { NO_LOGGING = 0, DIRECT, BUFFERED };

class Factory {
 public:
  virtual ~Factory() = default;

  virtual auto CreateImageProvider(const ImageProviderConfigData& image_provider_config, prometheus::Registry* registry)
      -> ImageProviderPtr                                                                       = 0;
  virtual auto CreateScanner(ImageProvider* image_provider, const ScannerCalibrationData& scanner_calibration,
                             const ScannerConfigurationData& scanner_configuration, const Fov& fov,
                             const core::scanner::JointProperties& joint_properties, ScannerOutputCB* scanner_output,
                             ImageLogger* logger, prometheus::Registry* registry) -> ScannerPtr = 0;

  virtual auto CreateImageLogger() -> ImageLoggerPtr = 0;
};

auto GetFactory() -> Factory*;

// For test
void SetFactoryGenerator(std::function<Factory*()> generator);
}  // namespace core::scanner
