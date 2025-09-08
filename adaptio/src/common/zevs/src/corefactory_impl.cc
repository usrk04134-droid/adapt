#include "corefactory_impl.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <zmq.hpp>

#include "../zevs_core.h"
#include "common/logging/application_log.h"
#include "coresocket_impl.h"
#include "coretimer_impl.h"
#include "eventloop_impl.h"
#include "message_impl.h"
#include "process_impl.h"
#include "timer_backend.h"

namespace zevs {

std::unique_ptr<CoreFactoryImpl> s_core_factory;

// Generator is typically a lambda capturing a Factory instance
// controlled by testcase.
std::function<CoreFactory *()> s_core_generator;

auto GetCoreFactory() -> CoreFactory * {
  if (s_core_generator) {
    return s_core_generator();
  }

  if (!s_core_factory) {
    s_core_factory = std::make_unique<CoreFactoryImpl>();
  }

  return s_core_factory.get();
}

// For test
// can set an empty std::function to release Factory instance
// captured in generator previously set.
void SetCoreFactoryGenerator(std::function<CoreFactory *()> generator) { s_core_generator = std::move(generator); }

auto CoreFactoryImpl::CreateContext() -> std::unique_ptr<Context> {
  auto context_impl = std::make_unique<ContextImpl>();
  return context_impl;
}

auto CoreFactoryImpl::CreateEventLoop(const std::string &unique_name) -> std::shared_ptr<EventLoop> {
  auto event_loop     = std::make_shared<EventLoopImpl>(unique_name);
  auto timer_endpoint = "inproc://" + unique_name + "_timer";
  auto core_socket    = GetCoreFactory()->CreateCoreSocket(*event_loop, SocketType::SUB, MessageType::PACKAGED);

  auto timer_backend = std::make_shared<TimerBackend>(timer_endpoint, core_socket);
  event_loop->SetTimer(timer_backend);
  return event_loop;
}

// Version without eventloop is used for sockets which are used for sending only,
// currently pub and pair
auto CoreFactoryImpl::CreateCoreSocket(SocketType socket_type, MessageType message_type) -> CoreSocketPtr {
  zmq::socket_type z_socket_type{};
  if (SocketType::PUB == socket_type) {
    z_socket_type = zmq::socket_type::pub;
  } else if (SocketType::PAIR == socket_type) {
    z_socket_type = zmq::socket_type::pair;
  } else {
    LOG_ERROR("createCoreSocket() without eventloop: incorrect SocketType");
    return {};
  }

  auto *ctx                = GetGlobalContext();
  auto internal_zmq_socket = std::make_unique<zmq::socket_t>(*ctx, z_socket_type);

  std::shared_ptr<CoreSocket> core_socket_impl =
      std::make_shared<CoreSocketImpl>(socket_type, message_type, std::move(internal_zmq_socket));

  return core_socket_impl;
}

auto CoreFactoryImpl::CreateCoreSocket(EventLoop &event_loop, SocketType socket_type, MessageType message_type)
    -> CoreSocketPtr {
  zmq::socket_type z_socket_type{};
  if (SocketType::SUB == socket_type) {
    z_socket_type = zmq::socket_type::sub;
  } else if (SocketType::PAIR == socket_type) {
    z_socket_type = zmq::socket_type::pair;
  } else {
    LOG_ERROR("createCoreSocket with event_loop: incorrect SocketType");
    return {};
  }

  auto &event_loop_impl = dynamic_cast<EventLoopImpl &>(event_loop);

  auto *ctx                     = GetGlobalContext();
  auto internal_zmq_socket      = std::make_unique<zmq::socket_t>(*ctx, z_socket_type);
  auto *internal_zmq_socket_ptr = internal_zmq_socket.get();
  std::shared_ptr<CoreSocketImpl> core_socket_impl =
      std::make_shared<CoreSocketImpl>(socket_type, message_type, std::move(internal_zmq_socket));
  event_loop_impl.RegisterCoreSocket(core_socket_impl, internal_zmq_socket_ptr);

  return core_socket_impl;
}

auto CoreFactoryImpl::CreateCoreTimer(EventLoop &event_loop) -> CoreTimerPtr {
  auto &eventloop_impl = dynamic_cast<EventLoopImpl &>(event_loop);
  return std::make_shared<CoreTimerImpl>(eventloop_impl.GetTimer());
}

auto CoreFactoryImpl::CreatePackagedMessage(uint32_t message_id, size_t size) -> MessagePtr {
  return std::make_unique<PackagedMessage>(message_id, size);
}

auto CoreFactoryImpl::CreateRawMessage(size_t size) -> MessagePtr { return std::make_unique<RawMessage>(size); }

}  // namespace zevs
