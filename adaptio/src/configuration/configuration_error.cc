
#include "configuration/configuration_error.h"

#include <string>
#include <system_error>

using configuration::ConfigurationErrorCode;

// Error code implementation
namespace {

struct ErrorCategory : std::error_category {
  auto name() const noexcept -> const char* final;          // NOLINT(*-use-nodiscard)
  auto message(int error_code) const -> std::string final;  // NOLINT(*-use-nodiscard)
  auto default_error_condition(int other) const noexcept    // NOLINT(*-use-nodiscard)
      -> std::error_condition final;                        // NOLINT(*-use-nodiscard)
};

auto ErrorCategory::name() const noexcept -> const char* { return "ConfigurationError"; }

auto ErrorCategory::message(int error_code) const -> std::string {
  switch (static_cast<ConfigurationErrorCode>(error_code)) {
    case ConfigurationErrorCode::NO_ERROR:
      return "No error";
    case ConfigurationErrorCode::CONFIGURATION_NO_DEFAULT_CONFIG_FILE:
      return "No default configuration file found";
    case ConfigurationErrorCode::CONFIGURATION_NO_USER_CONFIG_FILE:
      return "No user configuration file found";
    case ConfigurationErrorCode::CONFIGURATION_READ_FILE_ERROR:
      return "Not possible to read configuration file";
    case ConfigurationErrorCode::CONFIGURATION_WRITE_FILE_ERROR:
      return "Not possible to write to configurartion file";
    case ConfigurationErrorCode::CONFIGURATION_WRITE_FILE_MISSING:
      return "Configuration file missing";
    case ConfigurationErrorCode::CONFIGURATION_TAG_NOT_FOUND:
      return "Configuration tag not found";
    case ConfigurationErrorCode::CONFIGURATION_MISSING_CHECKSUM:
      return "Configuration file is missing the checksum";
    case ConfigurationErrorCode::CONFIGURATION_INVALID_CHECKSUM:
      return "The configuration file checksum is invalid";
  }
}

auto ErrorCategory::default_error_condition(int other) const noexcept -> std::error_condition {
  switch (static_cast<ConfigurationErrorCode>(other)) {
    default:
      return {other, *this};
  }
}

const ErrorCategory ERROR_CATEGORY{};

}  // namespace

[[maybe_unused]] auto configuration::make_error_code(ConfigurationErrorCode error_code)
    -> std::error_code {  // NOLINT(*-identifier-naming)
  return {static_cast<int>(error_code), ERROR_CATEGORY};
}
