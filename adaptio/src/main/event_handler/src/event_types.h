#pragma once

#include <fmt/core.h>
#include <yaml-cpp/emittermanip.h>

#include <optional>
#include <string>
#include <unordered_map>

#include "event_handler/event_codes.h"

namespace event {

auto const RESOLUTION_TYPE_WAIT             = "WAIT";
auto const RESOLUTION_TYPE_WAIT_AND_ACK     = "WAIT_AND_ACK";
auto const RESOLUTION_TYPE_ACKABLE          = "ACKABLE";
auto const RESOLUTION_TYPE_WAIT_OR_ACK      = "WAIT_OR_ACK";
auto const RESOLUTION_TYPE_RESTART_REQUIRED = "RESTART_REQUIRED";
auto const ACTION_PREVENT_START             = "PREVENT_START";
auto const ACTION_QUICKSTOP                 = "QUICKSTOP";
auto const ACTION_SUPER_QUICKSTOP           = "SUPER_QUICKSTOP";

enum class ResolutionType {
  WAIT,
  WAIT_AND_ACK,
  ACKABLE,
  WAIT_OR_ACK,
  RESTART_REQUIRED,
};

inline auto ResolutionTypeToString(ResolutionType rt) -> std::string {
  switch (rt) {
    case ResolutionType::WAIT:
      return RESOLUTION_TYPE_WAIT;
    case ResolutionType::WAIT_AND_ACK:
      return RESOLUTION_TYPE_WAIT_AND_ACK;
    case ResolutionType::ACKABLE:
      return RESOLUTION_TYPE_ACKABLE;
    case ResolutionType::WAIT_OR_ACK:
      return RESOLUTION_TYPE_WAIT_OR_ACK;
    case ResolutionType::RESTART_REQUIRED:
      return RESOLUTION_TYPE_RESTART_REQUIRED;
    default:
      break;
  }

  return "invalid";
}

inline auto ResolutionTypeFromString(const std::string& str) -> std::optional<ResolutionType> {
  static const std::unordered_map<std::string, ResolutionType> MAP = {
      {RESOLUTION_TYPE_WAIT,             ResolutionType::WAIT            },
      {RESOLUTION_TYPE_WAIT_AND_ACK,     ResolutionType::WAIT_AND_ACK    },
      {RESOLUTION_TYPE_ACKABLE,          ResolutionType::ACKABLE         },
      {RESOLUTION_TYPE_WAIT_OR_ACK,      ResolutionType::WAIT_OR_ACK     },
      {RESOLUTION_TYPE_RESTART_REQUIRED, ResolutionType::RESTART_REQUIRED},
  };

  auto it = MAP.find(str);

  return it != MAP.end() ? std::optional(it->second) : std::nullopt;
}

inline auto ResolutionTypeNeedAck(ResolutionType rt) -> bool {
  switch (rt) {
    case ResolutionType::WAIT_AND_ACK:
    case ResolutionType::ACKABLE:
    case ResolutionType::WAIT_OR_ACK:
      return true;
    case ResolutionType::WAIT:
    case ResolutionType::RESTART_REQUIRED:
    default:
      break;
  }

  return false;
}

inline auto ResolutionTypeNeedClear(ResolutionType rt) -> bool {
  switch (rt) {
    case ResolutionType::WAIT:
    case ResolutionType::WAIT_AND_ACK:
    case ResolutionType::WAIT_OR_ACK:
      return true;
    case ResolutionType::ACKABLE:
    case ResolutionType::RESTART_REQUIRED:
    default:
      break;
  }

  return false;
}

enum class EventStatus { ACTIVE, RESOLVED };

inline auto EventStatusToString(EventStatus status) -> std::string {
  switch (status) {
    case EventStatus::ACTIVE:
      return "active";
    case EventStatus::RESOLVED:
      return "resolved";
    default:
      return "unknown";
  }
}

enum class Action {
  PREVENT_START,
  QUICKSTOP,
  SUPER_QUICKSTOP,
};

inline auto ActionToString(Action action) -> std::string {
  switch (action) {
    case Action::PREVENT_START:
      return ACTION_PREVENT_START;
    case Action::QUICKSTOP:
      return ACTION_QUICKSTOP;
    case Action::SUPER_QUICKSTOP:
      return ACTION_SUPER_QUICKSTOP;
    default:
      break;
  }

  return "invalid";
}

inline auto ActionFromString(const std::string& str) -> std::optional<Action> {
  static const std::unordered_map<std::string, Action> MAP = {
      {ACTION_PREVENT_START,   Action::PREVENT_START  },
      {ACTION_QUICKSTOP,       Action::QUICKSTOP      },
      {ACTION_SUPER_QUICKSTOP, Action::SUPER_QUICKSTOP},
  };

  auto it = MAP.find(str);

  return it != MAP.end() ? std::optional(it->second) : std::nullopt;
}

struct Event {
 public:
  std::string version;
  Code code;
  std::string group;
  std::string title;
  std::string description;
  Action action;
  std::string severity;
  ResolutionType resolution_type;
  std::string source;
  std::string remarks;

  auto Valid() const& -> bool {
    return !version.empty() && version == "1.0" && !code.empty() && !severity.empty() && !source.empty() &&
           !title.empty() && !description.empty();
  };

  auto ToString() const& -> std::string {
    return fmt::format("code: {}, title: {}, action: {}, resolution_type: {}", code, title, ActionToString(action),
                       ResolutionTypeToString(resolution_type));
  };
};
}  // namespace event
