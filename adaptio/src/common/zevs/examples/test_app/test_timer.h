#pragma once

#include <cstdint>
#include <iostream>
#include <string>

#include "../../zevs_core.h"
#include "../../zevs_socket.h"

const uint32_t NUM_TIMEOUTS = 5;

class TimeTester {
 public:
  void ThreadEntry(const std::string &name) {
    event_loop_ = zevs::GetCoreFactory()->CreateEventLoop(name);
    timer_      = zevs::GetFactory()->CreateTimer(*event_loop_);
    InitLongTimer();
    InitShortTimer();

    event_loop_->Run();
  }

  void OnLongTimeout() {
    std::cout << "********** Long timeout\n";
    ++count_;
    if (count_ == NUM_TIMEOUTS) {
      timer_->Cancel(long_task_id_);
      timer_->Cancel(short_task_id_);
      event_loop_->Exit();
    }
  }

  void OnShortTimeout() { std::cout << "**** Short timeout\n"; }

  void InitLongTimer() {
    uint32_t long_duration_ms = 1000;
    long_task_id_ = timer_->RequestPeriodic(&TimeTester::OnLongTimeout, this, long_duration_ms, "long_timer");
  }

  void InitShortTimer() {
    uint32_t short_duration_ms = 333;
    short_task_id_ = timer_->RequestPeriodic(&TimeTester::OnShortTimeout, this, short_duration_ms, "short_timer");
  }

 private:
  zevs::EventLoopPtr event_loop_;
  zevs::TimerPtr timer_;
  uint32_t long_task_id_;
  uint32_t short_task_id_;
  uint32_t count_ = 0;
};
