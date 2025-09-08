#include "coretimer_impl.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "timer_internal.h"

namespace zevs {

CoreTimerImpl::CoreTimerImpl(const TimerInternalWeakPtr& timer_internal) : timer_internal_(timer_internal) {}

auto CoreTimerImpl::Request(uint32_t duration_ms, bool periodic, const std::string& task_name) -> uint32_t {
  auto on_timeout = [weak_this = weak_from_this(), periodic, task_name, this](uint32_t task_id) {
    if (weak_this.expired()) {
      // This CoreTimer has been deleted
      return true;  // true means handler is done and can be deleted.
    }
    if (handler_) {
      handler_(task_id, task_name);
    }

    if (periodic) {
      return false;  // handler is not done
    }
    return true;  // handler is done
  };

  if (timer_internal_.expired()) {
    return 0;
  }

  auto task_id        = timer_internal_.lock()->Request(on_timeout, duration_ms, periodic);
  task_info_[task_id] = {task_id, task_name, periodic};
  return task_id;
}

void CoreTimerImpl::Cancel(uint32_t task_id) {
  task_info_.erase(task_id);
  if (timer_internal_.expired()) {
    return;
  }

  timer_internal_.lock()->Cancel(task_id);
}

void CoreTimerImpl::SetHandler(std::function<void(uint32_t, const std::string&)> handler) { handler_ = handler; }

auto CoreTimerImpl::Describe() const -> std::string {
  std::string desc = "{";
  for (const auto& [_, task] : task_info_) {
    std::string periodic_or_once = (task.periodic) ? "periodic" : "once";
    desc += "{id=" + std::to_string(task.id) + " , name=" + task.name + ", " + periodic_or_once + "},";
  }
  desc += "}";

  return desc;
}

}  // namespace zevs
