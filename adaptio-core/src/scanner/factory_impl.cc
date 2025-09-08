#include "core/scanner/factory_impl.h"

#include <prometheus/registry.h>

#include <functional>
#include <memory>
#include <utility>

#include "core/image/tilted_perspective_camera.h"
#include "core/scanner/factory.h"
#include "core/scanner/image_logger.h"
#include "core/scanner/image_logger_impl.h"
#include "core/scanner/image_provider.h"
#include "core/scanner/image_provider/basler_camera.h"
#include "core/scanner/image_provider/simulation/camera_simulation.h"
#include "core/scanner/image_provider_configuration.h"
#include "core/scanner/joint_buffer/circular_joint_buffer.h"
#include "core/scanner/joint_model.h"
#include "core/scanner/joint_model/big_snake.h"
#include "core/scanner/scanner.h"
#include "core/scanner/scanner_calibration_configuration.h"
#include "core/scanner/scanner_configuration.h"
#include "core/scanner/scanner_impl.h"
#include "core/scanner/tiff_handler_impl.h"

namespace core::scanner {

using image::TiltedPerspectiveCamera;

std::unique_ptr<FactoryImpl> s_factory;

// Generator is typically a lambda capturing a Factory instance
// controlled by testcase.
std::function<Factory*()> s_generator;

auto GetFactory() -> Factory* {
  if (s_generator) {
    return s_generator();
  }

  if (!s_factory) {
    s_factory = std::make_unique<FactoryImpl>();
  }

  return s_factory.get();
}

// For test
// can set an empty std::function to release Factory instance
// captured in generator previously set.
void SetFactoryGenerator(std::function<Factory*()> generator) { s_generator = std::move(generator); }

auto FactoryImpl::CreateScanner(ImageProvider* image_provider, const ScannerCalibrationData& scanner_calibration,
                                const ScannerConfigurationData& scanner_configuration, const Fov& fov,
                                const core::scanner::JointProperties& joint_properties, ScannerOutputCB* scanner_output,
                                ImageLogger* logger, prometheus::Registry* registry) -> ScannerPtr {
  auto joint_buffer = std::make_unique<CircularJointBuffer>();
  image::TiltedPerspectiveCameraProperties camera_properties;
  camera_properties.config_calib = scanner_calibration;
  camera_properties.config_fov   = fov;

  auto camera_model = std::make_unique<TiltedPerspectiveCamera>(camera_properties);
  return std::make_unique<ScannerImpl>(
      image_provider, std::move(joint_buffer), [](bool state) {}, scanner_output,
      JointModelPtr(new joint_model::BigSnake(joint_properties, scanner_configuration, std::move(camera_model))),
      logger, registry);
}

auto FactoryImpl::CreateImageProvider(const ImageProviderConfigData& image_provider_config,
                                      prometheus::Registry* registry) -> ImageProviderPtr {
  if (image_provider_config.image_provider == ImageProviderType::BASLER) {
    return std::make_unique<BaslerCamera>(image_provider_config.basler_config, image_provider_config.fov, registry);
  }

  return std::make_unique<CameraSimulation>(image_provider_config.sim_config, true /*loop images for tests*/);
}

auto FactoryImpl::CreateImageLogger() -> ImageLoggerPtr {
  auto tiff_handler = std::make_unique<TiffHandlerImpl>();

  return std::make_unique<ImageLoggerImpl>(std::move(tiff_handler));
}
}  // namespace core::scanner
