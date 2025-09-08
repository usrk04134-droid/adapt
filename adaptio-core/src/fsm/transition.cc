#include <string>
#include <utility>

#include "core/fsm/state.h"

using core::fsm::Transition;

Transition::Transition(std::string name) : name_(std::move(name)), qualifier_(TransitionQualifier::ON_END) {}

auto Transition::Evaluate() const -> bool {
  switch (qualifier_) {
    case ON_END: {
      return parent_->AtEndState() && condition_(parent_);
    } break;
    case ALWAYS: {
      return condition_(parent_);
    } break;
  }
  return false;
}

auto Transition::GetTarget() const -> State * { return target_; }

auto Transition::GetName() const -> std::string { return name_; }

void Transition::SetParent(core::fsm::State *state) { parent_ = state; }

void Transition::SetTarget(State *state) { target_ = state; }

void Transition::SetCondition(const core::fsm::Condition &condition, core::fsm::TransitionQualifier qualifier) {
  condition_ = condition;
  qualifier_ = qualifier;
}
