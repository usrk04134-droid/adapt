#pragma once

#include <chrono>
#include <cstdint>
#include <cstring>
#include <thread>

#include "../../zevs_socket.h"
#include "test_if.h"

class TestPublisher {
 public:
  void ThreadEntry() {
    // Note, objects containing sockets must be created in the
    // same thread where they are uesd (e.g. Publisher)
    Init();

    SendStructAMessages();
    SendStructBMessage1();
    SendStructBMessage2();
  }

  void Init() {
    socket_ = zevs::GetFactory()->CreatePubSocket();
    socket_->Bind("inproc://#1");

    // Sleep a few ms to allow subscriber to connect.
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  void SendStructAMessages() {
    for (uint32_t i = 0; i < 3; ++i) {
      test_if::A a;
      a.i1 = i;
      a.i2 = 6;
      strcpy(a.s1, "test");
      socket_->SendWithEnvelope("topic1", a);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  void SendStructBMessage1() {
    test_if::B b;
    b.b1 = true;
    b.i1 = 42;

    socket_->SendWithEnvelope("topic1", b);
  }

  void SendStructBMessage2() {
    test_if::B b;
    b.b1 = false;
    b.i1 = 7;

    socket_->SendWithEnvelope("topic2", b);
  }

  void Run() { my_thread_ = std::thread(&TestPublisher::ThreadEntry, this); }

  ~TestPublisher() { my_thread_.join(); }

 private:
  zevs::SocketPtr socket_;
  std::thread my_thread_;
};
