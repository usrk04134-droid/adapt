#pragma once

#include "activity_status.h"
#include "common/logging/application_log.h"

namespace coordination {

class ActivityStatusImpl : public ActivityStatus {
 public:
  auto Get() const -> ActivityStatusE override { return value_; }
  void Set(ActivityStatusE value) override {
    LOG_INFO("Activity status {} -> {}", ActivityStatusToString(value_), ActivityStatusToString(value));
    value_ = value;
    if (on_update_) {
      on_update_();
    }
  }
  auto IsIdle() const -> bool override { return value_ == ActivityStatusE::IDLE; }
  auto ToString() const -> std::string override { return ActivityStatusToString(value_); }
  void Subscribe(std::function<void()> on_update) override { on_update_ = on_update; };

 private:
  ActivityStatusE value_;
  std::function<void()> on_update_;
};

}  // namespace coordination
