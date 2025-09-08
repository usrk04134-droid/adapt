#pragma once

#include <filesystem>
#include <string>

#include "conf_factory.h"
#include "configuration/conf_file_handler.h"
#include "configuration/converter.h"

namespace configuration {

class FactoryImpl : public Factory {
 public:
  auto CreateConverter(const std::string& tag, const std::filesystem::path& file) -> ConverterPtr override;

  auto CreateFileHandler() -> FileHandlerPtr override;
};

}  // namespace configuration
