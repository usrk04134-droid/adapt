#include <memory>
#include <string>
#include <system_error>
#include <utility>

#include "core/fsm/state.h"

using core::fsm::TransitionBuilder;
using core::fsm::TransitionBuilderErrorCode;

namespace {
struct ErrorCategory : std::error_category {
  auto name() const noexcept -> const char * final;         // NOLINT(*-use-nodiscard)
  auto message(int error_code) const -> std::string final;  // NOLINT(*-use-nodiscard)
  auto default_error_condition(int other) const noexcept    // NOLINT(*-use-nodiscard)
      -> std::error_condition final;                        // NOLINT(*-use-nodiscard)
};

auto ErrorCategory::name() const noexcept -> const char * { return "ConfigurationError"; }

auto ErrorCategory::message(int error_code) const -> std::string {
  switch (static_cast<TransitionBuilderErrorCode>(error_code)) {
    case TransitionBuilderErrorCode::NO_ERROR:
      return "No error";
    case TransitionBuilderErrorCode::NO_TARGET_SET:
      return "No target set for transition";
  }
}

auto ErrorCategory::default_error_condition(int other) const noexcept -> std::error_condition {
  switch (static_cast<TransitionBuilderErrorCode>(other)) {
    case TransitionBuilderErrorCode::NO_TARGET_SET:
      return std::errc::invalid_argument;
    default:
      return {other, *this};
  }
}

const ErrorCategory ERROR_CATEGORY{};

}  // namespace

// NOLINTNEXTLINE(*-identifier-naming)
[[maybe_unused]] auto core::fsm::make_error_code(TransitionBuilderErrorCode error_code) -> std::error_code {
  return {static_cast<int>(error_code), ERROR_CATEGORY};
}

TransitionBuilder::TransitionBuilder(const std::string &name) { transition_ = std::make_unique<Transition>(name); }

auto TransitionBuilder::SetTarget(State *state) -> TransitionBuilder & {
  transition_->SetTarget(state);
  return *this;
}

auto TransitionBuilder::SetCondition(const core::fsm::Condition &condition, core::fsm::TransitionQualifier qualifier)
    -> TransitionBuilder & {
  transition_->SetCondition(condition, qualifier);
  return *this;
}

auto TransitionBuilder::Finalize() -> boost::outcome_v2::result<std::unique_ptr<Transition>> {
  if (transition_->target_ == nullptr) {
    return TransitionBuilderErrorCode::NO_TARGET_SET;
  }

  return std::move(transition_);
}
