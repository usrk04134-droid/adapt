#pragma once

#include "../zevs_core.h"
#include "../zevs_socket.h"

namespace zevs {

class FactoryImpl : public Factory {
 public:
  auto CreatePubSocket() -> SocketPtr override;
  auto CreateSubSocket(EventLoop& event_loop) -> SocketPtr override;
  auto CreatePairSocket() -> SocketPtr override;
  auto CreatePairSocket(EventLoop& event_loop) -> SocketPtr override;
  auto CreateTimer(EventLoop& event_loop) -> TimerPtr override;
};

}  // namespace zevs
