#pragma once

#include <optional>
#include <string>

#include "event_handler/event_codes.h"

namespace event {

class EventHandler {
 public:
  virtual ~EventHandler() = default;

  virtual void RaiseEvent(const Code& code, std::optional<std::string> detail) = 0;
  virtual void ClearEvent(const Code& code)                                    = 0;
  virtual void SendEvent(const Code& code, std::optional<std::string> detail)  = 0;
};

}  // namespace event
