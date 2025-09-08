#include "eventloop_impl.h"

#include <zmq.h>

#include <cerrno>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <zmq.hpp>

#include "common/logging/application_log.h"
#include "coresocket_impl.h"
#include "process_impl.h"
#include "timer_internal.h"

namespace zevs {

void ExitEventLoop(const std::string &eventloop_name) {
  auto *ctx = GetGlobalContext();
  if (!ctx) {
    LOG_ERROR("exitEventLoop({}): invalid context", eventloop_name);
    return;
  }

  zmq::socket_t exit_send_socket{*ctx, zmq::socket_type::pair};
  exit_send_socket.connect("inproc://" + eventloop_name + "_exit");

  zmq::message_t msg;
  exit_send_socket.send(msg, zmq::send_flags::none);
}

EventLoopImpl::EventLoopImpl(const std::string &unique_name) : unique_name_(unique_name) {
  if (!GetGlobalContext()) {
    LOG_ERROR("EventLoopImpl({}): invalid context", unique_name);
  }
}

void EventLoopImpl::RegisterCoreSocket(std::weak_ptr<CoreSocketImpl> coresocket, zmq::socket_t *z_socket) {
  sockets_.push_back(std::make_pair(coresocket, z_socket));
}

void EventLoopImpl::SetTimer(const TimerInternalSharedPtr &timer) { timer_ = timer; }

auto EventLoopImpl::GetTimer() const -> TimerInternalWeakPtr { return timer_; }

void EventLoopImpl::Run() {
  LOG_DEBUG("Starting eventloop {}", unique_name_);

  // Create socket for receiving the exit event.
  // It does bind. The other side does connect when
  // exitEventLoop() is called.
  auto *ctx = GetGlobalContext();
  zmq::socket_t exit_recv_socket{*ctx, zmq::socket_type::pair};
  exit_recv_socket.bind("inproc://" + unique_name_ + "_exit");

  while (true) {
    //  These vectors are rebuilt every iteration since there is no
    //  deregistration of sockets for simplicity. The performance
    //  cost of this was measured to be relatively small compared to the
    //  send/receive logging in coresocket_impl.cc
    std::vector<zmq::pollitem_t> poll_items;
    std::vector<std::weak_ptr<CoreSocketImpl>> core_sockets;

    auto iter = sockets_.begin();
    while (iter != sockets_.end()) {
      auto core_socket = iter->first;
      if (core_socket.expired()) {
        LOG_DEBUG("Eventloop::run(), socket expired");
        iter = sockets_.erase(iter);  // iter now at next element or end
      } else {
        auto *zsocket = iter->second;
        poll_items.push_back({*zsocket, 0, ZMQ_POLLIN, 0});
        core_sockets.push_back(core_socket);
        ++iter;
      }
    }

    // Add poll item for exit socket last
    poll_items.push_back({exit_recv_socket, 0, ZMQ_POLLIN, 0});
    const auto index_of_exit_item = poll_items.size() - 1;

    try {
      zmq::poll(poll_items);
    } catch (zmq::error_t &exception) {
      if (EINTR == exception.num()) {
        // This occurs if program has registered a signal handler for exit.
        // E.g. when ctrl-c is used to terminate the program
        // and when program is terminated by system-test
        continue;
      }
      throw;
    }

    for (auto i = 0; i < (int)index_of_exit_item; ++i) {
      if (poll_items[i].revents & ZMQ_POLLIN) {
        // locking the weak ptr once again since previous callbacks in this loop
        // could have caused core_socket object deletes
        auto core_socket = core_sockets[i].lock();
        if (core_socket) {
          core_socket->OnEvent();
        }
      }
    }

    // Finally check for an exit event
    if (poll_items[index_of_exit_item].revents & ZMQ_POLLIN) {
      break;
    }
  }

  LOG_DEBUG("Exit eventloop {}", unique_name_);
}

void EventLoopImpl::Exit() { ExitEventLoop(unique_name_); }

}  // namespace zevs
