#pragma once

#include <iostream>
#include <string>
#include <thread>

#include "../../zevs_core.h"
#include "../../zevs_socket.h"
#include "interfaces.h"

class Messenger {
 public:
  void OnExternalTestS1(test_if::TestS1 data) {
    std::cout << "external TestS1 - "
              << "i1:" << data.i1 << ", f1:" << data.f1 << ", b1:" << data.b1 << ", u1:" << data.u1 << "\n";

    SendMessage2();
  }

  void OnExternalTestS2(test_if::TestS2 data) {
    std::cout << "external TestS2 - "
              << "d1:" << data.d1 << ", i1:" << data.i1 << "\n";

    SendMessage1();
    event_loop_->Exit();
  }

  void SendMessage1() {
    test_if::TestS1 test;
    test.i1 = 66;
    test.f1 = 3.14;
    test.b1 = true;
    test.u1 = 99;

    socket_->Send(test);
  }

  void SendMessage2() {
    test_if::TestS2 test;
    test.d1 = -3.14;
    test.i1 = -999;

    socket_->Send(test);
  }

  // Eventloop, timeer and socket handling
  void ThreadEntry(const std::string name) {
    event_loop_ = zevs::GetCoreFactory()->CreateEventLoop(name);

    socket_ = zevs::GetFactory()->CreatePairSocket(*event_loop_);
    socket_->Connect(external_endpoint_);
    socket_->Serve(&Messenger::OnExternalTestS1, this);
    socket_->Serve(&Messenger::OnExternalTestS2, this);
    event_loop_->Run();
  }

  void Run(const std::string name) { my_thread_ = std::thread(&Messenger::ThreadEntry, this, name); }

  explicit Messenger(const std::string& external_endpoint) : external_endpoint_(external_endpoint) {}
  ~Messenger() { my_thread_.join(); }

 private:
  std::thread my_thread_;
  zevs::EventLoopPtr event_loop_;
  zevs::CoreSocketPtr external_socket_;
  zevs::SocketPtr socket_;
  std::string external_endpoint_;
};
