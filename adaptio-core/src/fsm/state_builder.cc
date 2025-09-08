
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <utility>

#include "core/fsm/state.h"

using core::fsm::StateBuilder;
using core::fsm::StateBuilderErrorCode;
using core::fsm::TransitionBuilder;

namespace {
struct ErrorCategory : std::error_category {
  auto name() const noexcept -> const char* final;          // NOLINT(*-use-nodiscard)
  auto message(int error_code) const -> std::string final;  // NOLINT(*-use-nodiscard)
  auto default_error_condition(int other) const noexcept    // NOLINT(*-use-nodiscard)
      -> std::error_condition final;                        // NOLINT(*-use-nodiscard)
};

auto ErrorCategory::name() const noexcept -> const char* { return "ConfigurationError"; }

auto ErrorCategory::message(int error_code) const -> std::string {
  switch (static_cast<StateBuilderErrorCode>(error_code)) {
    case StateBuilderErrorCode::NO_ERROR:
      return "No error";
    case StateBuilderErrorCode::INVALID_POWER_SOURCE_CONFIGURATION:
      return "Invalid power source configuration";
    case StateBuilderErrorCode::EMPTY_STATE:
      return "Empty state";
  }
}

auto ErrorCategory::default_error_condition(int other) const noexcept -> std::error_condition {
  switch (static_cast<StateBuilderErrorCode>(other)) {
    case StateBuilderErrorCode::INVALID_POWER_SOURCE_CONFIGURATION:
    case StateBuilderErrorCode::EMPTY_STATE:
      return std::errc::invalid_argument;
    default:
      return {other, *this};
  }
}

const ErrorCategory ERROR_CATEGORY{};

}  // namespace

[[maybe_unused]] auto core::fsm::make_error_code(StateBuilderErrorCode error_code)
    -> std::error_code {  // NOLINT(*-identifier-naming)
  return {static_cast<int>(error_code), ERROR_CATEGORY};
}

StateBuilder::StateBuilder(const std::string& state_name)
    : state_(std::make_unique<State>(state_name)), allow_empty_(false) {}

auto StateBuilder::OnEnter(const Action& action) -> StateBuilder& {
  state_->AddOnEnter(action);
  return *this;
}

auto StateBuilder::OnTick(const Action& action) -> StateBuilder& {
  state_->AddOnTick(action);
  return *this;
}

auto StateBuilder::OnExit(const Action& action) -> StateBuilder& {
  state_->AddOnExit(action);
  return *this;
}

auto StateBuilder::SetTimeout(std::chrono::duration<double> time_limit) -> StateBuilder& {
  state_->SetTimeout(time_limit);
  return *this;
}

auto StateBuilder::OnTimeout(const Action& action) -> StateBuilder& {
  state_->AddOnTimeout(action);
  return *this;
}

auto StateBuilder::AllowEmpty() -> StateBuilder& {
  allow_empty_ = true;
  return *this;
}

auto StateBuilder::AddSuperTransition(std::unique_ptr<Transition> transition) -> StateBuilder& {
  state_->AddSuperTransition(std::move(transition));
  return *this;
}

auto StateBuilder::AddTransition(std::unique_ptr<Transition> transition) -> StateBuilder& {
  state_->AddTransition(std::move(transition));
  return *this;
}

auto StateBuilder::AddState(std::unique_ptr<State> state) -> StateBuilder& {
  state_->AddState(std::move(state));
  return *this;
}

auto StateBuilder::SetEndState(core::fsm::State* state) -> StateBuilder& {
  state_->SetEndState(state);
  return *this;
}

auto StateBuilder::Finalize() -> boost::outcome_v2::result<std::unique_ptr<State>> {
  if (!allow_empty_ && state_->transitions_.empty() && state_->states_.empty() && state_->on_enter_.empty() &&
      state_->on_tick_.empty() && state_->on_exit_.empty() && state_->on_timeout_.empty()) {
    return StateBuilderErrorCode::EMPTY_STATE;
  }
  return std::move(state_);
}

#ifndef DOCTEST_CONFIG_DISABLE
// NOLINTBEGIN(readability-magic-number)

#include <doctest/doctest.h>

#include <boost/thread/thread.hpp>
#include <cstdint>

TEST_SUITE("Test StateBuilder") {
  TEST_CASE("Trivial state building") {
    auto state = StateBuilder("State").AllowEmpty().Finalize().value();

    CHECK_NE(state.get(), nullptr);
    CHECK_EQ(state->GetName(), "State");
  }

  TEST_CASE("Test actions") {
    bool enter = false, tick = false, exit = false;

    auto state = StateBuilder("State")
                     .OnEnter(ACTION(&enter) { enter = true; })
                     .OnTick(ACTION(&tick) { tick = true; })
                     .OnExit(ACTION(&exit) { exit = true; })
                     .Finalize()
                     .value();

    state->Enter();
    CHECK_EQ(enter, true);
    CHECK_EQ(tick, false);
    CHECK_EQ(exit, false);

    state->Tick();
    CHECK_EQ(enter, true);
    CHECK_EQ(tick, true);
    CHECK_EQ(exit, false);

    state->Exit();
    CHECK_EQ(enter, true);
    CHECK_EQ(tick, true);
    CHECK_EQ(exit, true);
  }

  TEST_CASE("Test state switching") {}

  TEST_CASE("Test timeout") {
    bool timed_out = false;

    std::chrono::milliseconds timeout_length(100);
    auto state = StateBuilder("State")
                     .SetTimeout(timeout_length)
                     .OnTimeout(ACTION(&timed_out) { timed_out = true; })
                     .Finalize()
                     .value();

    state->Enter();
    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    state->Tick();
    CHECK_EQ(timed_out, false);

    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    state->Tick();
    CHECK_EQ(timed_out, false);

    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    state->Tick();
    CHECK_EQ(timed_out, true);
  }

  TEST_CASE("Test state switch on timeout") {
    bool timed_out = false;
    std::chrono::milliseconds timeout_length(100);

    auto child_state_2 = StateBuilder("Child state 2").AllowEmpty().Finalize().value();

    auto child_state_1 = StateBuilder("Child state 1")
                             .SetTimeout(timeout_length)
                             .OnTimeout(ACTION(&timed_out) { timed_out = true; })
                             .AddTransition(TransitionBuilder("Child state 1 timeout")
                                                .SetCondition(CONDITION() { return state->IsTimedOut(); })
                                                .SetTarget(child_state_2.get())
                                                .Finalize()
                                                .value())
                             .Finalize()
                             .value();

    auto main_state = StateBuilder("Main state")
                          .AddState(std::move(child_state_1))
                          .AddState(std::move(child_state_2))
                          .Finalize()
                          .value();

    {
      main_state->Enter();
      boost::this_thread::sleep(boost::posix_time::milliseconds(10));
      main_state->Tick();
      CHECK_EQ(main_state->GetCompleteName(), "Main state - Child state 1");
    }

    {
      main_state->Enter();
      boost::this_thread::sleep(boost::posix_time::milliseconds(10));
      main_state->Tick();
      CHECK_EQ(main_state->GetCompleteName(), "Main state - Child state 1");
    }

    {
      main_state->Enter();
      boost::this_thread::sleep(boost::posix_time::milliseconds(100));
      main_state->Tick();
      CHECK_EQ(main_state->GetCompleteName(), "Main state - Child state 2");
    }
  }

  TEST_CASE("Test passing data") {
    struct Data {
      int32_t i = 0;
    };

    Data data = {.i = 23};

    auto state = StateBuilder("State")
                     .OnEnter(ACTION() {
                       auto* data = static_cast<Data*>(state->GetData().value());
                       data->i    = 42;
                     })
                     .Finalize()
                     .value();

    CHECK_EQ(state->GetData(), std::nullopt);

    state->SetData(&data);

    CHECK_NE(state->GetData(), std::nullopt);

    CHECK_EQ(data.i, 23);

    state->Enter();

    CHECK_EQ(data.i, 42);
  }

  TEST_CASE("Test TransitionQualifier::Always") {
    bool stop_    = false;
    bool stopped_ = false;
    bool inited_  = false;

    bool* stop    = &stop_;
    bool* stopped = &stopped_;
    bool* inited  = &inited_;

    auto process_state = StateBuilder("Process state").AllowEmpty().Finalize().value();

    auto init_state = StateBuilder("Init state")
                          .AddTransition(TransitionBuilder("Inited")
                                             .SetCondition(CONDITION(inited) { return *inited; })
                                             .SetTarget(process_state.get())
                                             .Finalize()
                                             .value())
                          .Finalize()
                          .value();

    auto stop_state = StateBuilder("Stop state").OnEnter(ACTION(stopped) { *stopped = true; }).Finalize().value();

    auto root_state =
        StateBuilder("Root")
            .AddSuperTransition(TransitionBuilder("Stop sequence")
                                    .SetCondition(
                                        CONDITION(stop) { return *stop; }, core::fsm::TransitionQualifier::ALWAYS)
                                    .SetTarget(stop_state.get())
                                    .Finalize()
                                    .value())
            .AddState(std::move(init_state))
            .AddState(std::move(process_state))
            .AddState(std::move(stop_state))
            .Finalize()
            .value();

    root_state->Enter();
    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    root_state->Tick();
    CHECK_EQ(root_state->GetCompleteName(), "Root - Init state");

    inited_ = true;
    root_state->Tick();
    CHECK_EQ(root_state->GetCompleteName(), "Root - Process state");

    root_state->Tick();
    root_state->Tick();
    root_state->Tick();
    CHECK_EQ(root_state->GetCompleteName(), "Root - Process state");

    stop_ = true;
    root_state->Tick();
    root_state->Tick();
    root_state->Tick();
    CHECK_EQ(root_state->GetCompleteName(), "Root - Stop state");
    CHECK_EQ(stopped_, true);
  }

  TEST_CASE("Test ticking with several \"always\"-transitions") {
    bool child_1_to_2_     = false;
    bool child_1_1_to_1_2_ = false;
    bool global_1_         = false;
    bool global_2_         = false;

    bool* child_1_to_2     = &child_1_to_2_;
    bool* child_1_1_to_1_2 = &child_1_1_to_1_2_;
    bool* global_1         = &global_1_;
    bool* global_2         = &global_2_;

    auto child_1_2 = StateBuilder("Child state 1, 2").AllowEmpty().Finalize().value();

    auto child_1_1 = StateBuilder("Child state 1, 1")
                         .AddTransition(TransitionBuilder("Child state 1, 1 to 2")
                                            .SetCondition(CONDITION(child_1_1_to_1_2) { return *child_1_1_to_1_2; })
                                            .SetTarget(child_1_2.get())
                                            .Finalize()
                                            .value())
                         .Finalize()
                         .value();

    auto child_2 = StateBuilder("Child state 2").AllowEmpty().Finalize().value();

    auto child_1 = StateBuilder("Child state 1")
                       .AddTransition(TransitionBuilder("Child state 1 to 2")
                                          .SetCondition(CONDITION(child_1_to_2) { return *child_1_to_2; })
                                          .SetTarget(child_2.get())
                                          .Finalize()
                                          .value())
                       .AddSuperTransition(TransitionBuilder("GLOBAL - Child state 1 1 to 2 2")
                                               .SetCondition(
                                                   CONDITION(global_2) { return *global_2; }, core::fsm::ALWAYS)
                                               .SetTarget(child_1_2.get())
                                               .Finalize()
                                               .value())
                       .AddState(std::move(child_1_1))
                       .AddState(std::move(child_1_2))
                       .Finalize()
                       .value();

    auto main_state = StateBuilder("Main state")
                          .AddTransition(TransitionBuilder("GLOBAL - Child state 1 to 2")
                                             .SetCondition(
                                                 CONDITION(global_1) { return *global_1; }, core::fsm::ALWAYS)
                                             .SetTarget(child_2.get())
                                             .Finalize()
                                             .value())
                          .AddState(std::move(child_1))
                          .AddState(std::move(child_2))
                          .Finalize()
                          .value();

    main_state->Enter();
    main_state->Tick();
    CHECK_EQ(main_state->GetCompleteName(), "Main state - Child state 1 - Child state 1, 1");

    global_2_ = true;
    main_state->Tick();
    CHECK_EQ(main_state->GetCompleteName(), "Main state - Child state 1 - Child state 1, 2");
  }
}

// NOLINTEND(readability-magic-number)
#endif
