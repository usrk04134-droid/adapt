
#include <utility>

#include "scanner/core/scanner_calibration_configuration.h"
#include "scanner/core/scanner_configuration.h"
#include "scanner/core/src/scanner.h"
#include "scanner/image_logger/image_logger.h"
#include "scanner/scanner_factory.h"

// The stub constructor inserts itself to be returned when the application
// calls getFactory. The destructor restores normal behaviour.
// The stub should be created/destroyed for each application testcase since it
// has state which should be cleared for each testcase.
class StubCoreScannerFactory : public scanner::ScannerFactory {
 public:
  explicit StubCoreScannerFactory(scanner::ScannerPtr scanner, scanner::image_logger::ImageLoggerPtr image_logger)
      : scanner_(std::move(scanner)), image_logger_(std::move(image_logger)) {
    auto factory_generator = [this]() { return this; };
    scanner::SetFactoryGenerator(factory_generator);
  }
  ~StubCoreScannerFactory() override { SetFactoryGenerator(std::function<scanner::ScannerFactory*()>{}); }

  auto CreateImageProvider(const scanner::image_provider::ImageProviderConfigData& /*image_provider_configuration*/,
                           prometheus::Registry* /*registry*/) -> scanner::image_provider::ImageProviderPtr override {
    // Maybe split the factory in an imageprovider factory and a scanner factory?
    // Then each type of stub factory can be constructed with the respective mock.
    return nullptr;
  }
  auto CreateScanner(scanner::image_provider::ImageProvider* /*image_provider*/,
                     const scanner::ScannerCalibrationData& /*scanner_calibration*/,
                     const scanner::ScannerConfigurationData& /*scanner_configuration*/,
                     const scanner::image_provider::Fov& /*fov*/,
                     const scanner::joint_model::JointProperties& /*joint_properties*/,
                     scanner::ScannerOutputCB* scanner_output, scanner::image_logger::ImageLogger* /*logger*/,
                     prometheus::Registry* /*registry*/) -> scanner::ScannerPtr override {
    scanner_output_ = scanner_output;
    return std::move(scanner_);
  }

  auto CreateImageLogger() -> scanner::image_logger::ImageLoggerPtr override { return std::move(image_logger_); };

  auto GetScannerOutput() -> scanner::ScannerOutputCB* { return scanner_output_; }

 private:
  scanner::ScannerPtr scanner_;
  scanner::image_logger::ImageLoggerPtr image_logger_;
  scanner::ScannerOutputCB* scanner_output_;
};
