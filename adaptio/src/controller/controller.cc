#include "controller/controller.h"

#include <string>
#include <system_error>

namespace {

using controller::ControllerErrorCode;

struct ErrorCategory : std::error_category {
  auto name() const noexcept -> const char* final;          // NOLINT(*-use-nodiscard)
  auto message(int error_code) const -> std::string final;  // NOLINT(*-use-nodiscard)
  auto default_error_condition(int other) const noexcept    // NOLINT(*-use-nodiscard)
      -> std::error_condition final;                        // NOLINT(*-use-nodiscard)
};

auto ErrorCategory::name() const noexcept -> const char* { return "ControllerError"; }

auto ErrorCategory::message(int error_code) const -> std::string {
  switch (static_cast<ControllerErrorCode>(error_code)) {
    case ControllerErrorCode::NO_ERROR:
      return "No error";
    case ControllerErrorCode::FAILED_TO_CONNECT:
      return "Failed to connect";
    case ControllerErrorCode::ALREADY_CONNECTED:
      return "Already connected";
    case ControllerErrorCode::DISCONNECTED:
      return "Controller is disconnected";
    case ControllerErrorCode::INPUT_ERROR:
      return "An error occurred when reading inputs";
    case ControllerErrorCode::OUTPUT_ERROR:
      return "An error occurred when writing outputs";
    case ControllerErrorCode::UNEXPECTED_ERROR:
      return "Unexpected error";
    case ControllerErrorCode::MAX_NUMBER_OF_ATTEMPTS:
      return "Max number of connection attempts";
  }
}

auto ErrorCategory::default_error_condition(int other) const noexcept -> std::error_condition {
  switch (static_cast<ControllerErrorCode>(other)) {
    case ControllerErrorCode::FAILED_TO_CONNECT:
      return std::errc::connection_aborted;
    case ControllerErrorCode::ALREADY_CONNECTED:
      return std::errc::already_connected;
    case ControllerErrorCode::DISCONNECTED:
      return std::errc::not_connected;
    case ControllerErrorCode::INPUT_ERROR:
      return std::errc::io_error;
    case ControllerErrorCode::OUTPUT_ERROR:
      return std::errc::io_error;
    case ControllerErrorCode::UNEXPECTED_ERROR:
      return std::errc::protocol_error;
    case ControllerErrorCode::MAX_NUMBER_OF_ATTEMPTS:
      return std::errc::connection_aborted;
    default:
      return {other, *this};
  }
}

const ErrorCategory ERROR_CATEGORY{};

}  // namespace

[[maybe_unused]] auto controller::make_error_code(controller::ControllerErrorCode error_code)
    -> std::error_code {  // NOLINT(*-identifier-naming)
  return {static_cast<int>(error_code), ERROR_CATEGORY};
}
