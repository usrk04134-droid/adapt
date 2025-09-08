#pragma once
#include <memory>
#include <string>

namespace configuration {

class FileHandler {
 public:
  virtual ~FileHandler()                                                             = default;
  virtual auto ReadFile(const std::string& file) -> std::string                      = 0;
  virtual auto WriteFile(const std::string& file, const std::string& string) -> bool = 0;
  virtual auto FileExist(const std::string& file) -> bool                            = 0;
  virtual auto GetAbsParent(const std::string& file) -> std::string                  = 0;
  virtual void SetWritePermission(const std::string& file)                           = 0;
};
using FileHandlerPtr = std::shared_ptr<FileHandler>;
}  // namespace configuration
