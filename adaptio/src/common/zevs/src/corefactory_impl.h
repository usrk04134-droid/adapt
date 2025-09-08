#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "../zevs_core.h"

namespace zevs {

class CoreFactoryImpl : public CoreFactory {
 public:
  auto CreateContext() -> std::unique_ptr<Context> override;
  auto CreateEventLoop(const std::string& unique_name) -> EventLoopPtr override;
  auto CreateCoreSocket(SocketType socket_type, MessageType message_type) -> CoreSocketPtr override;
  auto CreateCoreSocket(EventLoop& event_loop, SocketType socket_type, MessageType message_type)
      -> CoreSocketPtr override;
  auto CreateCoreTimer(EventLoop& event_loop) -> CoreTimerPtr override;
  auto CreatePackagedMessage(uint32_t message_id, size_t size) -> MessagePtr override;
  auto CreateRawMessage(size_t size) -> MessagePtr override;
};

}  // namespace zevs
