#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace zevs {

//
// ---------- process scope ----------
class Context {
 public:
  virtual ~Context() = default;
};

using ContextPtr = std::unique_ptr<Context>;

//
// ---------- thread scope ----------
class EventLoop {
 public:
  virtual ~EventLoop() = default;
  virtual void Run()   = 0;
  virtual void Exit()  = 0;
};

using EventLoopPtr = std::shared_ptr<EventLoop>;

void ExitEventLoop(const std::string &eventloop_name);

//
// ---------- Message ----------
enum class MessageType { PACKAGED, RAW };

class Message {
 public:
  virtual ~Message()                       = default;
  virtual auto Id() const -> uint32_t      = 0;
  virtual auto Size() const -> size_t      = 0;
  virtual auto Data() const -> void *      = 0;
  virtual auto Type() const -> MessageType = 0;
};

using MessagePtr = std::unique_ptr<Message>;

//
// ---------- Core Socket ----------
enum class SocketType { PUB, SUB, PAIR };

class CoreSocket {
 public:
  virtual ~CoreSocket()                                                          = default;
  virtual void Bind(const std::string &endpoint)                                 = 0;
  virtual void Connect(const std::string &endpoint)                              = 0;
  virtual void SetHandler(std::function<void(MessagePtr)> handler)               = 0;
  virtual void Send(MessagePtr message)                                          = 0;
  virtual void SendWithEnvelope(const std::string &envelope, MessagePtr message) = 0;
  virtual void SetFilter(const std::string &envelope)                            = 0;
  virtual auto Describe() const -> std::string                                   = 0;
};

using CoreSocketPtr = std::shared_ptr<CoreSocket>;

//
// ---------- Core Timer ----------
class CoreTimer {
 public:
  virtual ~CoreTimer() = default;

  virtual void SetHandler(std::function<void(uint32_t, const std::string &)> handler)                             = 0;
  virtual auto Request(uint32_t duration_ms, bool periodic, const std::string &task_name) -> /*task_id*/ uint32_t = 0;
  virtual void Cancel(uint32_t task_id)                                                                           = 0;
  virtual auto Describe() const -> std::string                                                                    = 0;
};

using CoreTimerPtr = std::shared_ptr<CoreTimer>;

//
// ---------- Core Factory ----------
class CoreFactory {
 public:
  virtual ~CoreFactory() = default;

  // Process scope:
  virtual auto CreateContext() -> ContextPtr = 0;

  // Thread scope:
  virtual auto CreateEventLoop(const std::string &name) -> EventLoopPtr = 0;

  // CoreSocket
  virtual auto CreateCoreSocket(SocketType socket_type, MessageType message_type) -> CoreSocketPtr = 0;
  virtual auto CreateCoreSocket(EventLoop &event_loop, SocketType socket_type, MessageType message_type)
      -> CoreSocketPtr = 0;

  // CoreTimer
  virtual auto CreateCoreTimer(EventLoop &event_loop) -> CoreTimerPtr = 0;

  // Message
  virtual auto CreatePackagedMessage(uint32_t message_id, size_t size) -> MessagePtr = 0;
  virtual auto CreateRawMessage(size_t size) -> MessagePtr                           = 0;
};

auto GetCoreFactory() -> CoreFactory *;

// For test
void SetCoreFactoryGenerator(std::function<CoreFactory *()> generator);

//
// ---------- Logg Interface ----------
// The purpose of this class is to avoid inclusion of actual Logg header
// in the template API since it will spread the Logg header depenency.
class Logging {
 public:
  virtual ~Logging()                                                                        = default;
  virtual void LogInfo(const std::string &source, const std::string &msg, uint32_t number)  = 0;
  virtual void LogError(const std::string &source, const std::string &msg, uint32_t number) = 0;
};

using LoggingPtr = std::shared_ptr<Logging>;

}  // namespace zevs
