#pragma once

#include <filesystem>
#include <functional>
#include <string>

#include "common/data/data_value.h"
#include "conf_file_handler.h"
#include "converter.h"

namespace configuration {

class Factory {
 public:
  virtual ~Factory() = default;

  virtual auto CreateConverter(const std::string& tag, const std::filesystem::path& file) -> ConverterPtr = 0;

  virtual auto CreateFileHandler() -> FileHandlerPtr = 0;
};

auto GetFactory() -> Factory*;

// For test
void SetFactoryGenerator(std::function<Factory*()> generator);

}  // namespace configuration
