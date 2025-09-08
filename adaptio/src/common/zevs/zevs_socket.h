#pragma once

#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "zevs_core.h"

namespace zevs {

// About ZEVS (Zeromq EVentS):
// The main feature of ZEVS is to provide an application thread with a centralized
// receive-loop (poll) and dispatch of multiple callbacks from several sockets to
// different application objects in the thread. Timeout events are also supported
// (and handled in the same thread as receive events), for example to provide
// cyclic HW control or supervision of time-limited activities.
// These features make it possible to only share data as messages between
// threads. Another important feature is the possibility to "stub out"
// the actual zmq framework in application tests. There are two API
// layers, one core API which is pure virtual and a convenience template API.
// The minimal, stable, pure virtual API allows mocking and the possibility to
// provide a library in the future.​ An additional template API layer provides
// specialized, expandable and convenient use in applications.​
//
// About the Socket API (this header):
// The purpose of the template API in this header is to provide easy send/serve
// of application types (message structs). Callbacks (serve) are provided in
// the form of class member function pointers for reliability reasons. Since
// Socket instances are referenced with shared pointers (SocketPtr), such callbacks
// tend to be safe since the callbacks usually target member functions in the class
// owning the shared pointer to the Socket. When the owning class instance is deleted,
// the Socket is also deleted and callbacks will not be attempted. With callbacks
// in the form of std::function objects the risk for callbacks to deleted
// instances (crashes) would be much higher.
//
// For convenient testing of classes using this API, there is a test support header
// (zevs_test_support.h) containing a Mocket class and stubbed factories. It can be
// used to dispatch/receive events in testcases to avoid sending/receiving actual
// zmq buffers. This way the tests can be single threaded and therefore much simpler
// and deterministic.
//
// The current version of this convenience/template API supports only
// send/receive of trivially copyable strucs (mem_copyable).
// To send other data, use the CoreSocket API. (zevs_core.h)
//
// If in the future there is a need for dynamic data or transport across a network,
// ZEVS can be adapted for use with serialization frameworks like Protocol buffers.
// The recommended approach in that case is to let the template API handle the
// generated Protocol buffers datatypes instead of the structs used in the current
// API. The message_id can be defined as an enum in the message in the .proto files.
// Pair socket may not be the best socket type for transport across a network so
// additional socket types can be investigated.

template <class T>
concept mem_copyable = std::is_trivially_copyable_v<T>;

class Socket {
 public:
  template <typename C, typename Data>
  using Func = void (C::*)(Data);

  // API methods:
  void Bind(const std::string& endpoint);

  void Connect(const std::string& endpoint);

  template <typename C, typename Data>
    requires mem_copyable<Data>
  void Serve(Func<C, Data> callback_function, C* callback_instance);

  template <typename Data>
    requires mem_copyable<Data>
  void Send(Data data);

  template <typename Data>
    requires mem_copyable<Data>
  void SendWithEnvelope(const std::string& envelope, Data data);

  void SetFilter(const std::string& envelope);

  auto Describe() const -> std::string;
  // End of API methods

  Socket(SocketType socket_type, CoreSocketPtr core_socket, LoggingPtr logging);

 private:
  void OnMessage(MessagePtr msg);

  SocketType socket_type_;
  CoreSocketPtr core_socket_;
  LoggingPtr logging_;
  std::unordered_map<uint32_t, std::function<void(MessagePtr)>> message_handlers_;
};

using SocketPtr = std::shared_ptr<Socket>;

class Timer {
 public:
  template <typename C>
  using Func = void (C::*)();

  // API methods:
  // Note: The purpose of task_name is to simplify tests and improve logs
  // The Request methods return a task_id which can be used to cancel timeouts
  template <typename C>
  auto Request(Func<C> function, C* instance, uint32_t duration_ms, const std::string& task_name) -> uint32_t;

  template <typename C>
  auto RequestPeriodic(Func<C> function, C* instance, uint32_t duration_ms, const std::string& task_name) -> uint32_t;

  void Cancel(uint32_t task_id);
  auto Describe() const -> std::string;
  // End of API methods

  explicit Timer(CoreTimerPtr core_timer, LoggingPtr logging);

 private:
  void OnTimeout(uint32_t task_id, const std::string& task_name);
  CoreTimerPtr core_timer_;
  LoggingPtr logging_;
  std::unordered_map<uint32_t, std::function<bool()>> handlers_;
};

using TimerPtr = std::shared_ptr<Timer>;

class Factory {
 public:
  virtual ~Factory() = default;

  virtual auto CreatePubSocket() -> SocketPtr                       = 0;
  virtual auto CreateSubSocket(EventLoop& event_loop) -> SocketPtr  = 0;
  virtual auto CreatePairSocket() -> SocketPtr /* For send only*/   = 0;
  virtual auto CreatePairSocket(EventLoop& event_loop) -> SocketPtr = 0;
  virtual auto CreateTimer(EventLoop& event_loop) -> TimerPtr       = 0;
};

auto GetFactory() -> Factory*;

// For test
void SetFactoryGenerator(std::function<Factory*()> generator);

//
// -------------- Implementation details below --------------
//

template <typename C, typename Data>
  requires mem_copyable<Data>
void Socket::Serve(Func<C, Data> callback_function, C* callback_instance) {
  auto handler = [callback_function, callback_instance](MessagePtr message) {
    auto* data       = message->Data();
    auto* typed_data = static_cast<Data*>(data);

    (callback_instance->*callback_function)(*typed_data);
  };

  auto message_id = static_cast<uint32_t>(Data::Metadata::MESSAGE_ID);
  if (message_handlers_.contains(message_id)) {
    logging_->LogError(core_socket_->Describe(), "serve(): message_id {} already exists",
                       message_id);  // overwrite handler
  }
  message_handlers_[message_id] = handler;
}

template <typename Data>
  requires mem_copyable<Data>
void Socket::Send(Data data) {
  auto message_id     = static_cast<uint32_t>(Data::Metadata::MESSAGE_ID);
  auto message        = GetCoreFactory()->CreatePackagedMessage(message_id, sizeof(data));
  auto* destination_p = message->Data();
  std::memcpy(destination_p, &data, sizeof(data));  // write the struct to the message buffer
  core_socket_->Send(std::move(message));
}

template <typename Data>
  requires mem_copyable<Data>
void Socket::SendWithEnvelope(const std::string& envelope, Data data) {
  auto message_id     = static_cast<uint32_t>(Data::Metadata::MESSAGE_ID);
  auto message        = GetCoreFactory()->CreatePackagedMessage(message_id, sizeof(data));
  auto* destination_p = message->Data();
  std::memcpy(destination_p, &data, sizeof(data));  // write the struct to the message buffer
  core_socket_->SendWithEnvelope(envelope, std::move(message));
}

void inline Socket::SetFilter(const std::string& envelope) { core_socket_->SetFilter(envelope); }

inline Socket::Socket(SocketType socket_type, CoreSocketPtr core_socket, LoggingPtr logging)
    : socket_type_(socket_type), core_socket_(core_socket), logging_(std::move(logging)) {
  if ((SocketType::SUB == socket_type_) || (SocketType::PAIR == socket_type_)) {
    auto message_handler = [this](MessagePtr msg) { this->OnMessage(std::move(msg)); };
    core_socket_->SetHandler(message_handler);
  }
}

inline void Socket::Bind(const std::string& endpoint) { core_socket_->Bind(endpoint); }

inline void Socket::Connect(const std::string& endpoint) { core_socket_->Connect(endpoint); }

inline void Socket::OnMessage(MessagePtr msg) {
  auto message_id = msg->Id();
  auto iter       = message_handlers_.find(message_id);
  if (iter == message_handlers_.end()) {
    logging_->LogInfo(core_socket_->Describe(), "Unknown message_id", message_id);
    return;
  }

  auto handler = iter->second;
  handler(std::move(msg));
}

inline auto Socket::Describe() const -> std::string { return core_socket_->Describe(); }

inline Timer::Timer(CoreTimerPtr core_timer, LoggingPtr logging) : core_timer_(core_timer), logging_(logging) {
  auto handler = [this](uint32_t task_id, const std::string& task_name) { this->OnTimeout(task_id, task_name); };
  core_timer_->SetHandler(handler);
}

inline void Timer::OnTimeout(uint32_t task_id, const std::string& task_name) {
  if (!handlers_.contains(task_id)) {
    logging_->LogInfo("Timer::OnTimeout", "task_name=" + task_name + ", Unknown task_id", task_id);
    return;
  }

  // Taking a copy of the handler since the callback can be reentrant and
  // erase the handler from the map.
  auto handler = handlers_[task_id];
  auto done    = handler();
  if (done) {
    // This was a single timerout.
    handlers_.erase(task_id);
  }
}

template <typename C>
auto Timer::Request(Func<C> function, C* instance, uint32_t duration_ms, const std::string& task_name) -> uint32_t {
  auto handler = [function, instance]() {
    (instance->*function)();
    return true;  // Handler is done after timeout, for single timeouts.
  };
  auto task_id       = core_timer_->Request(duration_ms, false, task_name);
  handlers_[task_id] = handler;
  return task_id;
}

template <typename C>
auto Timer::RequestPeriodic(Func<C> function, C* instance, uint32_t duration_ms, const std::string& task_name)
    -> uint32_t {
  auto handler = [function, instance]() {
    (instance->*function)();
    return false;  // Handler is not done after timeout, for periodic timeouts.
  };
  auto task_id       = core_timer_->Request(duration_ms, true, task_name);
  handlers_[task_id] = handler;
  return task_id;
}

inline void Timer::Cancel(uint32_t task_id) {
  core_timer_->Cancel(task_id);
  handlers_.erase(task_id);
}

inline auto Timer::Describe() const -> std::string { return core_timer_->Describe(); }

}  // namespace zevs
