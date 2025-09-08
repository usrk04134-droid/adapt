#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "../zevs_core.h"
#include "coresocket_impl.h"
#include "timer_internal.h"
#include "zmq.hpp"

namespace zevs {

class EventLoopImpl : public EventLoop {
 public:
  explicit EventLoopImpl(const std::string &unique_name);

  // EventLoop
  void Run() override;
  void Exit() override;

  // Used by factory
  void RegisterCoreSocket(std::weak_ptr<CoreSocketImpl> coresocket, zmq::socket_t *z_socket);
  void SetTimer(const TimerInternalSharedPtr &timer);
  auto GetTimer() const -> TimerInternalWeakPtr;  // see note on weak_ptr below

 private:
  std::string unique_name_;
  std::vector<std::pair<std::weak_ptr<CoreSocketImpl>, zmq::socket_t *>> sockets_;

  // Note regarding TimerInternal:
  // The instance is owned by the EventLoop and used by all
  // CoreTimers related to the eventloop. To make sure
  // the TimerInternal instance (and timer thread) is deleted
  // when the EventLoop is deleted, the CoreTimers keep weak
  // pointers to the TimerInternal instance.
  TimerInternalSharedPtr timer_;
};

}  // namespace zevs
