#include "factory_impl.h"

#include <functional>
#include <memory>
#include <utility>

#include "../zevs_core.h"
#include "../zevs_socket.h"
#include "log_impl.h"

namespace zevs {

std::unique_ptr<FactoryImpl> s_factory;

// Generator is typically a lambda capturing a Factory instance
// controlled by testcase.
std::function<Factory*()> s_generator;

auto GetFactory() -> Factory* {
  if (s_generator) {
    return s_generator();
  }

  if (!s_factory) {
    s_factory = std::make_unique<FactoryImpl>();
  }

  return s_factory.get();
}

// For test
// can set an empty std::function to release Factory instance
// captured in generator previously set.
void SetFactoryGenerator(std::function<Factory*()> generator) { s_generator = std::move(generator); }

auto FactoryImpl::CreatePubSocket() -> SocketPtr {
  auto core_socket = GetCoreFactory()->CreateCoreSocket(SocketType::PUB, MessageType::PACKAGED);
  auto logging     = std::make_shared<LoggingImpl>();
  return std::make_shared<Socket>(SocketType::PUB, core_socket, logging);
}

auto FactoryImpl::CreatePairSocket() -> SocketPtr {
  auto core_socket = GetCoreFactory()->CreateCoreSocket(SocketType::PAIR, MessageType::PACKAGED);
  auto logging     = std::make_shared<LoggingImpl>();
  return std::make_shared<Socket>(SocketType::PAIR, core_socket, logging);
}

auto FactoryImpl::CreateSubSocket(EventLoop& event_loop) -> SocketPtr {
  auto core_socket = GetCoreFactory()->CreateCoreSocket(event_loop, SocketType::SUB, MessageType::PACKAGED);
  auto logging     = std::make_shared<LoggingImpl>();
  return std::make_shared<Socket>(SocketType::SUB, core_socket, logging);
}

auto FactoryImpl::CreatePairSocket(EventLoop& event_loop) -> SocketPtr {
  auto core_socket = GetCoreFactory()->CreateCoreSocket(event_loop, SocketType::PAIR, MessageType::PACKAGED);
  auto logging     = std::make_shared<LoggingImpl>();
  return std::make_shared<Socket>(SocketType::PAIR, core_socket, logging);
}

auto FactoryImpl::CreateTimer(EventLoop& event_loop) -> TimerPtr {
  auto core_timer = GetCoreFactory()->CreateCoreTimer(event_loop);
  auto logging    = std::make_shared<LoggingImpl>();
  return std::make_shared<Timer>(core_timer, logging);
}

}  // namespace zevs
