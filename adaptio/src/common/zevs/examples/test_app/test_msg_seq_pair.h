#pragma once

#include <cstddef>
#include <iostream>
#include <string>
#include <thread>

#include "../../zevs_core.h"
#include "../../zevs_socket.h"
#include "test_if.h"

class BothPeers {
 public:
  void ThreadEntry(const std::string name) {
    auto event_loop = zevs::GetCoreFactory()->CreateEventLoop(name);

    // bind / connect first endpoint§
    std::string endpoint1 = "inproc://msg_seq_test_ep1";
    auto bind_socket1     = zevs::GetFactory()->CreatePairSocket(*event_loop);
    bind_socket1->Bind(endpoint1);
    auto connect_socket1 = zevs::GetFactory()->CreatePairSocket(*event_loop);
    connect_socket1->Connect(endpoint1);

    // bind / connect second endpoint§
    std::string endpoint2 = "inproc://msg_seq_test_ep2";
    auto bind_socket2     = zevs::GetFactory()->CreatePairSocket(*event_loop);
    bind_socket2->Bind(endpoint2);
    auto connect_socket2 = zevs::GetFactory()->CreatePairSocket(*event_loop);
    connect_socket2->Connect(endpoint2);

    bind_socket1->Serve(&BothPeers::OnA, this);
    bind_socket2->Serve(&BothPeers::OnA, this);

    // Send 10 messages from socket1 and 1 from socket 2.
    // Observe the sequence in received messages.
    // Is the sequence due to zmq or the way poll is used?
    // Note that this is within a single thread.
    for (size_t i = 0; i < 10; ++i) {
      test_if::A a;
      a.i1 = 1;
      connect_socket1->Send(a);
    }

    test_if::A a;
    a.i1 = 2;
    connect_socket2->Send(a);

    event_loop->Run();
  }

  void OnA(test_if::A a) { std::cout << "Received message from: " << a.i1 << "\n"; }

  void Run(const std::string name) { my_thread_ = std::thread(&BothPeers::ThreadEntry, this, name); }

  ~BothPeers() { my_thread_.join(); }

 private:
  std::thread my_thread_;
};
