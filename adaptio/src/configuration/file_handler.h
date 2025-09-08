#pragma once

#include <string>

#include "conf_file_handler.h"

namespace configuration {

class FileHandlerImpl : public FileHandler {
 public:
  auto ReadFile(const std::string& file) -> std::string override;
  auto WriteFile(const std::string& file, const std::string& string) -> bool override;
  auto FileExist(const std::string& file) -> bool override;
  auto GetAbsParent(const std::string& file) -> std::string override;
  void SetWritePermission(const std::string& file) override;
  static auto GetAbsPath(const std::string& file) -> std::string;
};

}  // namespace configuration
