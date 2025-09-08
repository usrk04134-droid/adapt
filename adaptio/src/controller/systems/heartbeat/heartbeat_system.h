#pragma once

#include <chrono>
#include <functional>

#include "common/clock_functions.h"
#include "controller/controller_data.h"
#include "controller/systems/system.h"

namespace controller {

using CallbackFunction = std::function<void()>;
using ValidateFunction = std::function<bool()>;

class HeartbeatSystem : public System {
 public:
  explicit HeartbeatSystem(ValidateFunction validate_heartbeat, CallbackFunction heartbeat_lost_callback,
                           clock_functions::SteadyClockNowFunc steady_clock_now_func);

  void Apply() override;

 private:
  AdaptioInput* input_;
  AdaptioOutput* output_;
  std::chrono::time_point<std::chrono::steady_clock> heartbeat_last_changed_ = {};
  ValidateFunction validate_heartbeat_;
  CallbackFunction heartbeat_lost_callback_;
  clock_functions::SteadyClockNowFunc steady_clock_now_func_;
};

}  // namespace controller
