#pragma once

#include "controller/controller_data.h"

namespace controller {

// Implement by controller
class WeldSystemServerObserver {
 public:
  virtual ~WeldSystemServerObserver() = default;

  virtual void OnPowerSourceOutput(uint32_t index, PowerSourceOutput const& data) = 0;
};

// Use from controller
class WeldSystemServer {
 public:
  virtual ~WeldSystemServer() = default;

  virtual void OnPowerSourceInput(uint32_t index, PowerSourceInput const& data) = 0;
};

}  // namespace controller
