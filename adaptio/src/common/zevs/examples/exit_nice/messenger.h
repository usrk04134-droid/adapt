#pragma once

#include <cstdint>
#include <iostream>
#include <string>

#include "../../zevs_core.h"
#include "../../zevs_socket.h"

class Messenger {
 public:
  // Eventloop, timer and socket handling
  void ThreadEntry(const std::string name) {
    event_loop_ = zevs::GetCoreFactory()->CreateEventLoop(name);
    InitTimer();

    event_loop_->Run();
  }

  void OnTimeout() { std::cout << "Timeout\n"; }

  void InitTimer() {
    uint32_t duration_ms = 1000;
    timer_               = zevs::GetFactory()->CreateTimer(*event_loop_);
    timer_->RequestPeriodic(&Messenger::OnTimeout, this, duration_ms, "messenger_poll");
  }

 private:
  zevs::EventLoopPtr event_loop_;
  zevs::TimerPtr timer_;
};
