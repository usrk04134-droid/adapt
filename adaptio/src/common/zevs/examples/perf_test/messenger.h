#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>

#include "../../zevs_core.h"
#include "../../zevs_socket.h"

namespace test_if {

struct A {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x010000000 };
  uint32_t i1;
  uint32_t i2;
};

}  // namespace test_if

class Messenger {
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

    socket_->Serve(&Messenger::OnA, this);

    // Let connectside start the ping-pong sequence
    if ("ConnectSide" == name) {
      test_if::A start_msg{0, 0};
      socket_->Send(start_msg);
    }

    event_loop_->Run();
  }

  void OnA(test_if::A /*data*/) {
    socket_->Send(test_if::A{7, 42});
    if (target_no_msg_ == ++recv_count_) {
      socket_.reset();
      event_loop_->Exit();
    }
  }

  void Run(const std::string name) { my_thread_ = std::thread(&Messenger::ThreadEntry, this, name); }

  Messenger(size_t nr_to_send) : target_no_msg_(nr_to_send) {}
  ~Messenger() { my_thread_.join(); }

 private:
  zevs::EventLoopPtr event_loop_;
  zevs::SocketPtr socket_;
  std::thread my_thread_;
  uint32_t recv_count_ = 0;
  size_t target_no_msg_;
};
