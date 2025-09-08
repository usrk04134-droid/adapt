#pragma once
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "../zevs_core.h"
#include "timer_internal.h"
#include "timer_thread.h"

namespace zevs {

class TimerBackend : public TimerInternal {
 public:
  explicit TimerBackend(const std::string& timer_endpoint, CoreSocketPtr core_socket);
  ~TimerBackend();

  using OnTimeoutFunc = std::function<bool(uint32_t)>;

  // TimerInternal
  auto Request(OnTimeoutFunc on_timeout, uint32_t duration_ms, bool periodic) -> uint32_t override;
  void Cancel(uint32_t task_id) override;

 private:
  void OnTimeoutMessage(MessagePtr msg);
  void ExitTimerThread();
  std::string timer_endpoint_;
  CoreSocketPtr core_socket_;
  std::thread thread_;
  std::unique_ptr<TimerThread> timer_thread_;
  std::unordered_map<uint32_t, OnTimeoutFunc> handlers_;
};

using TimerBackendPtr = std::unique_ptr<TimerBackend>;

}  // namespace zevs
