#include "factory.h"

#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "common/logging/application_log.h"
#include "configuration/conf_factory.h"
#include "configuration/conf_file_handler.h"
#include "configuration/config_manager.h"
#include "configuration/converter.h"
#include "configuration/file_handler.h"
#include "scanner_calibration_converter.h"

namespace configuration {

std::unique_ptr<FactoryImpl> s_factory;

// Generator is typically a lambda capturing a Factory instance
// controlled by testcase.
std::function<Factory *()> s_generator;

auto GetFactory() -> Factory * {
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
void SetFactoryGenerator(std::function<Factory *()> generator) { s_generator = std::move(generator); }

auto FactoryImpl::CreateConverter(const std::string &tag, const std::filesystem::path &file) -> ConverterPtr {
  ConverterPtr converter;

  if (tag == TAG_SC) {
    converter = std::make_unique<ScannerCalibrationConverter>(tag, file);
  }

  else {
    LOG_ERROR("Unkonwn converter for tag: {}", tag);
  }

  return converter;
}

auto FactoryImpl::CreateFileHandler() -> FileHandlerPtr { return std::make_shared<FileHandlerImpl>(); }
}  // namespace configuration
