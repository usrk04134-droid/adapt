#pragma once
#include <list>
#include <sstream>
#include <string>

template <class T>
class StateMonitor {
 public:
  explicit StateMonitor() = default;
  ~StateMonitor()         = default;
  auto Clear() -> void { states_.clear(); }
  auto Add(T &state) -> void {
    if (states_.empty() || state != states_.back()) {
      states_.push_back(state);
    }
  }
  auto Equal(const std::list<T> &states) -> bool { return states == states_; }
  auto Dump() -> std::string {
    std::ostringstream dump_string;
    for (auto const &state : states_) {
      dump_string << state << " ";
    }
    return dump_string.str();
  }

 private:
  std::list<T> states_;
};
