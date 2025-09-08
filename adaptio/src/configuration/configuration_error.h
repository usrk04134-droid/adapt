#pragma once

#include <cstdint>
#include <system_error>

namespace configuration {

enum class ConfigurationErrorCode : uint32_t {
  NO_ERROR                             = 0,
  CONFIGURATION_NO_DEFAULT_CONFIG_FILE = 1,
  CONFIGURATION_NO_USER_CONFIG_FILE    = 2,
  CONFIGURATION_READ_FILE_ERROR        = 3,
  CONFIGURATION_WRITE_FILE_ERROR       = 4,
  CONFIGURATION_WRITE_FILE_MISSING     = 5,
  CONFIGURATION_TAG_NOT_FOUND          = 6,
  CONFIGURATION_MISSING_CHECKSUM       = 7,
  CONFIGURATION_INVALID_CHECKSUM       = 8,
};
// NOLINTNEXTLINE(*-identifier-naming)
[[maybe_unused]] auto make_error_code(ConfigurationErrorCode) -> std::error_code;

}  // namespace configuration

namespace std {
template <>
struct is_error_code_enum<configuration::ConfigurationErrorCode> : true_type {};
}  // namespace std
