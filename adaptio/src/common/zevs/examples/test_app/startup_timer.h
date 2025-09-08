#pragma once

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "../../zevs_core.h"
#include "../../zevs_socket.h"

class StartupTimer {
 public:
  void ThreadEntry(const std::string &name) {
    event_loop_    = zevs::GetCoreFactory()->CreateEventLoop(name);
    initial_timer_ = zevs::GetFactory()->CreateTimer(*event_loop_);
    exit_timer_    = zevs::GetFactory()->CreateTimer(*event_loop_);

    StartInitialTimer();
    event_loop_->Run();
  }

  void StartInitialTimer() {
    uint32_t duration_ms = 100;
    task_id_initial_     = initial_timer_->Request(&StartupTimer::OnInitialTimeout, this, duration_ms, "initial_timer");
  }

  void OnInitialTimeout() {
    StartExitTimer();

    timer_ = zevs::GetFactory()->CreateTimer(*event_loop_);

    for (auto duration_ms = 1; duration_ms < 11; ++duration_ms) {
      auto task_id = timer_->Request(&StartupTimer::OnTimeout, this, duration_ms, "multiple_single_timers");
      task_id_v.push_back(task_id);
    }
  }

  void StartExitTimer() {
    uint32_t duration_ms = 100;
    task_id_exit_        = exit_timer_->Request(&StartupTimer::OnExitTimeout, this, duration_ms, "exit_timer");
  }

  void OnTimeout() { ++count_; }

  void OnExitTimeout() {
    std::cout << "Timer count: " << count_ << "\n";
    event_loop_->Exit();
  }

 private:
  zevs::EventLoopPtr event_loop_;
  zevs::TimerPtr initial_timer_;
  uint32_t task_id_initial_;
  zevs::TimerPtr exit_timer_;
  uint32_t task_id_exit_;

  zevs::TimerPtr timer_;

  std::vector<uint32_t> task_id_v;
  uint32_t count_ = 0;
};
