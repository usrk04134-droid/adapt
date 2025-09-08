#pragma once

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include "../../zevs_core.h"
#include "../../zevs_socket.h"
#include "test_if.h"

// This example does not synchronize the startup
// still all messages are received. It uses pair sockets.

class Peer {
 public:
  void ThreadEntry(const std::string name) {
    event_loop_ = zevs::GetCoreFactory()->CreateEventLoop(name);

    socket_ = zevs::GetFactory()->CreatePairSocket(*event_loop_);

    std::string endpoint = "inproc://pair_test";
    if ("BindSide" == name) {
      socket_->Bind(endpoint);
    } else {
      socket_->Connect(endpoint);
    }

    socket_->Serve(&Peer::OnC, this);

    SendMessages();
    event_loop_->Run();

    std::cout << "Peer received " << recv_count_ << " messages\n";
  }

  void SendMessages() {
    for (size_t i = 0; i < nr_to_send_; ++i) {
      //      test_if::A a;
      //      socket->sendPtr(std::move(a));
      test_if::C c;
      socket_->Send(c);
    }
  }
  // void onA(test_if::A a) {
  void OnC(test_if::C c) {
    if (nr_to_send_ == ++recv_count_) {
      event_loop_->Exit();
    }
  }

  void Run(const std::string name) { my_thread_ = std::thread(&Peer::ThreadEntry, this, name); }

  Peer(size_t nr_to_send) : nr_to_send_(nr_to_send) {}
  ~Peer() { my_thread_.join(); }

 private:
  zevs::EventLoopPtr event_loop_;
  zevs::SocketPtr socket_;
  std::thread my_thread_;
  uint32_t recv_count_ = 0;
  size_t nr_to_send_;
};
