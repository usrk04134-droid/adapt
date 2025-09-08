#include "file_handler.h"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "common/logging/application_log.h"

namespace configuration {

auto FileHandlerImpl::ReadFile(const std::string& file) -> std::string {
  std::ifstream file_stream(GetAbsPath(file).c_str());
  if (!file_stream.is_open()) {
    // Todo: replace this with boost result
    return "";
  }

  std::string yaml_string(std::istreambuf_iterator<char>{file_stream}, {});
  file_stream.close();
  return yaml_string;
}

auto FileHandlerImpl::WriteFile(const std::string& file, const std::string& string) -> bool {
  std::ofstream file_stream(file);
  if (!file_stream.is_open()) {
    LOG_ERROR("Not able to open file: {}", file);
    return false;
  }

  file_stream << string;
  file_stream.close();
  return true;
}

auto FileHandlerImpl::FileExist(const std::string& file) -> bool { return std::filesystem::exists(GetAbsPath(file)); }

auto FileHandlerImpl::GetAbsParent(const std::string& file) -> std::string {
  return std::filesystem::absolute(GetAbsPath(file)).parent_path().string();
}

auto FileHandlerImpl::GetAbsPath(const std::string& file) -> std::string {
  auto res = file;

  if (file.contains("$")) {
    auto env = file.substr(1, file.find('/') - 1);
    res.replace(0, env.length() + 1, "/" + env);
    return res;
  }

  return std::filesystem::absolute(res);
}

void FileHandlerImpl::SetWritePermission(const std::string& file) {
  std::filesystem::permissions(
      file,
      std::filesystem::perms::owner_write | std::filesystem::perms::group_write | std::filesystem::perms::others_write,
      std::filesystem::perm_options::add);
}
}  // namespace configuration
