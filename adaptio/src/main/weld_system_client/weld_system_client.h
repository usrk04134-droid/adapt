#pragma once

#include <functional>

#include "weld_system_types.h"

namespace weld_system {
using OnGetWeldSystemData = std::function<void(WeldSystemId id, WeldSystemData const& status)>;
using OnStateChange       = std::function<void(WeldSystemId id, WeldSystemState state)>;

class WeldSystemClient {
 public:
  virtual ~WeldSystemClient() = default;

  virtual void GetWeldSystemData(WeldSystemId id, OnGetWeldSystemData on_response)        = 0;
  virtual void SetWeldSystemData(WeldSystemId id, WeldSystemSettings data)                = 0;
  virtual auto SubscribeWeldSystemStateChanges(OnStateChange on_state_change) -> uint32_t = 0;
  virtual void UnSubscribeWeldSystemStateChanges(uint32_t handle)                         = 0;
};

}  // namespace weld_system
