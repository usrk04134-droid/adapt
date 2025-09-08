#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>

#include "../zevs_core.h"
#include "timer_internal.h"

namespace zevs {

class CoreTimerImpl : public CoreTimer, public std::enable_shared_from_this<CoreTimerImpl> {
 public:
  explicit CoreTimerImpl(const TimerInternalWeakPtr& timer_internal);

  // CoreTimer
  void SetHandler(std::function<void(uint32_t, const std::string&)> handler) override;
  auto Request(uint32_t duration_ms, bool periodic, const std::string& task_name) -> uint32_t override;
  void Cancel(uint32_t task_id) override;
  auto Describe() const -> std::string override;

 private:
  // TaskInfo is currently used for description only
  struct TaskInfo {
    uint32_t id{};
    std::string name;
    bool periodic{};
  };
  TimerInternalWeakPtr timer_internal_;
  std::function<void(uint32_t /*task_id*/, const std::string& /*task_name*/)> handler_;
  std::map<uint32_t, TaskInfo> task_info_;
};

}  // namespace zevs
