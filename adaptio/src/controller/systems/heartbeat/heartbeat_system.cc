#include "heartbeat_system.h"

#include <bits/chrono.h>
#include <common/clock_functions.h>

#include <chrono>

#include "common/logging/application_log.h"

using controller::HeartbeatSystem;

HeartbeatSystem::HeartbeatSystem(ValidateFunction validate_heartbeat, CallbackFunction heartbeat_lost_callback,
                                 clock_functions::SteadyClockNowFunc steady_clock_now_func)
    : validate_heartbeat_(validate_heartbeat),
      heartbeat_lost_callback_(heartbeat_lost_callback),
      steady_clock_now_func_(steady_clock_now_func) {}

void HeartbeatSystem::Apply() {
  using namespace std::chrono_literals;

  if (validate_heartbeat_()) {
    heartbeat_last_changed_ = steady_clock_now_func_();
  } else {
    if (steady_clock_now_func_() - heartbeat_last_changed_ > 500ms) {
      LOG_ERROR("Detected the PLC heartbeat lost");
      heartbeat_lost_callback_();
    }
  }
}
