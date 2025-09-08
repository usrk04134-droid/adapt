#include "core/fsm/state.h"

#include <fmt/core.h>

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "core/logging/application_log.h"

using core::fsm::State;

State::State(const std::string& name)
    : name_(name),
      super_transitions_(),
      transitions_(),
      states_(),
      current_state_(std::nullopt),
      end_state_(std::nullopt),
      on_enter_(),
      on_tick_(),
      on_exit_(),
      on_timeout_(),
      time_limit_(std::nullopt),
      enter_time_(),
      time_spent_(0),
      data_(std::nullopt) {}

void State::Enter() {
  Reset();

  for (auto& action : on_enter_) {
    action(this);
  }

  if (current_state_.has_value()) {
    current_state_.value()->Enter();
  }
}

void State::Tick() {
  time_spent_ = std::chrono::high_resolution_clock::now() - enter_time_;

  if (time_limit_.has_value() && time_spent_ > time_limit_.value()) {
    for (auto& action : on_timeout_) {
      action(this);
    }
  }

  for (auto& action : on_tick_) {
    action(this);
  }

  // Update internal state
  if (current_state_.has_value()) {
    auto next_state = UpdateSuperTransitions();

    if (next_state.has_value()) {
      current_state_.value()->Exit();
      SetState(next_state.value());
      current_state_.value()->Enter();
    } else {
      current_state_.value()->Tick();

      next_state = current_state_.value()->UpdateTransitions();

      if (next_state.has_value()) {
        current_state_.value()->Exit();
        SetState(next_state.value());
        current_state_.value()->Enter();
      }
    }
  }
}

void State::Exit() {
  for (auto& action : on_exit_) {
    action(this);
  }
}

void State::AddOnEnter(const Action& action) { on_enter_.emplace_back(action); }

void State::AddOnTick(const Action& action) { on_tick_.emplace_back(action); }

void State::AddOnExit(const Action& action) { on_exit_.emplace_back(action); }

void State::SetTimeout(std::chrono::duration<double> time_limit) { time_limit_ = time_limit; }

void State::AddOnTimeout(const core::fsm::Action& action) { on_timeout_.emplace_back(action); }

void State::AddSuperTransition(std::unique_ptr<Transition> transition) {
  transition->SetParent(this);
  super_transitions_.emplace_back(std::move(transition));
};

void State::AddTransition(std::unique_ptr<Transition> transition) {
  transition->SetParent(this);
  transitions_.emplace_back(std::move(transition));
};

auto State::UpdateSuperTransitions() const -> std::optional<State*> {
  for (const auto& transition : super_transitions_) {
    if (transition->Evaluate()) {
      return transition->GetTarget();
    }
  }

  return std::nullopt;
}

auto State::UpdateTransitions() -> std::optional<State*> {
  auto next_state = GetNextState();
  if (next_state.has_value()) {
    return next_state.value();
  }
  return std::nullopt;
}

auto State::GetName() const -> std::string { return name_; }

auto State::GetCompleteName() const -> std::string {
  std::string current_state_breadcrumbs = name_;

  auto sub_state = current_state_;
  while (sub_state.has_value()) {
    current_state_breadcrumbs.append(fmt::format(" - {}", sub_state.value()->GetName()));
    sub_state = sub_state.value()->GetState();
  }

  return current_state_breadcrumbs;
}

auto State::IsTimedOut() const -> bool { return time_limit_.has_value() && time_spent_ > time_limit_.value(); }

auto State::GetTimeSpent() const -> std::chrono::duration<double> { return time_spent_; }

auto State::AtEndState() -> bool { return end_state_ == std::nullopt || current_state_ == end_state_.value(); }

void State::SetData(std::optional<void*> data) { data_ = data; }

auto State::GetData() const -> std::optional<void*> { return data_; }

void State::AddState(std::unique_ptr<State> state) { states_.emplace_back(std::move(state)); }

auto State::GetState() const -> std::optional<State*> { return current_state_; }

void State::SetState(State* new_state) {
  LOG_DEBUG("State transition ({}): {} -> {}", GetName(),
            current_state_.has_value() ? current_state_.value()->GetName() : "None", new_state->GetName());

  current_state_ = new_state;
}

void State::SetEndState(std::optional<State*> end_state) { end_state_ = end_state; }

auto State::GetEndState() -> std::optional<State*> { return end_state_; }

void State::Reset() {
  enter_time_ = std::chrono::high_resolution_clock::now();
  if (!states_.empty()) {
    SetState(states_[0].get());
  }
}

auto State::GetNextState() const -> std::optional<State*> {
  for (const auto& transition : transitions_) {
    if (transition->Evaluate()) {
      return transition->GetTarget();
    }
  }

  return std::nullopt;
}
