#pragma once
#include <cstdint>
#include <functional>
#include <memory>

namespace zevs {

class TimerInternal {
 public:
  virtual ~TimerInternal() = default;

  using OnTimeoutFunc = std::function<bool(uint32_t)>;
  virtual auto Request(OnTimeoutFunc on_timeout, uint32_t duration_ms, bool periodic) -> uint32_t = 0;
  virtual void Cancel(uint32_t task_id)                                                           = 0;
};

using TimerInternalSharedPtr = std::shared_ptr<TimerInternal>;
using TimerInternalWeakPtr   = std::weak_ptr<TimerInternal>;

}  // namespace zevs
