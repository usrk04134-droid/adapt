#pragma once
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include "../../zevs_core.h"
#include "../../zevs_socket.h"
#include "test_if.h"

class TestSubscriber {
 public:
  void ThreadEntry() {
    // Note, objects containing sockets must be created in the
    // same thread where they are uesd (e.g. Publisher)
    event_loop_ = zevs::GetCoreFactory()->CreateEventLoop(event_loop_name_);

    socket_ = zevs::GetFactory()->CreateSubSocket(*event_loop_);
    socket_->Connect("inproc://#1");

    socket_->Serve(&TestSubscriber::OnA, this);
    socket_->Serve(&TestSubscriber::OnB, this);
    socket_->SetFilter("topic1");  // topic2 messages will be dropped

    InitTimer();

    event_loop_->Run();
  }

  void OnA(test_if::A a) {
    std::cout << "onA(), received struct test_if::A: {" << a.i1 << ", " << a.i2 << ", " << a.s1 << "}\n";
  }

  void OnB(test_if::B b) { std::cout << "onB(), received struct test_if::B: {" << b.b1 << ", " << b.i1 << "}\n"; }

  void OnTimeout() { std::cout << "TestSubscriber::onTimeout()\n"; }

  void InitTimer() {
    uint32_t duration_ms = 30;
    timer_               = zevs::GetFactory()->CreateTimer(*event_loop_);
    timer_->RequestPeriodic(&TestSubscriber::OnTimeout, this, duration_ms, "subscriber_timer");
  }

  void Run() { my_thread_ = std::thread(&TestSubscriber::ThreadEntry, this); }

  TestSubscriber(const std::string event_loop_name) : event_loop_name_(event_loop_name) {}
  ~TestSubscriber() { my_thread_.join(); }

 private:
  std::string event_loop_name_;
  zevs::EventLoopPtr event_loop_;
  zevs::SocketPtr socket_;
  zevs::TimerPtr timer_;
  std::thread my_thread_;
};
