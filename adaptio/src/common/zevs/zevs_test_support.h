#pragma once

#include <fmt/core.h>

#include <algorithm>
#include <boost/log/trivial.hpp>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <source_location>
#include <string>
#include <utility>

#include "common/logging/application_log.h"
#include "zevs_core.h"

namespace zevs {

class Mocket;
class MocketMessage;
enum class Endpoint { INVALID, BIND, CONNECT };

struct EndpointInfo {
  Endpoint method = Endpoint::INVALID;
  std::string id;
  std::weak_ptr<Mocket> mocket;
};

class Mocket : public CoreSocket, public std::enable_shared_from_this<Mocket> {
 public:
  using Location = std::source_location;

  // The API consists of the following methods:
  template <typename Data>
  auto Receive(Location loc = Location::current()) -> std::optional<Data>;
  auto Receive(const std::function<bool(const MessagePtr &)> &fn_match, Location loc = Location::current())
      -> MessagePtr;

  auto ReceiveMessage(Location loc = Location::current()) -> MessagePtr;
  auto ReceiveMessageNoLog(Location loc = Location::current()) -> MessagePtr;  // For test-fixtures

  template <typename Data>
  void Dispatch(Data data, Location loc = Location::current());

  void DispatchMessage(MessagePtr message, Location loc = Location::current());
  void DispatchMessageNoLog(MessagePtr message, Location loc = Location::current());  // For test-fixtures

  auto Queued() const -> size_t { return message_queue_.size(); }

  using DispatchObserverT = std::function<void(const Mocket &, Location)>;
  void SetDispatchObserver(DispatchObserverT observer);
  // End of API methods

  using EndpointObserverT = std::function<void(EndpointInfo)>;
  Mocket(SocketType socket_type, const EndpointObserverT &observer);

  // CoreSocket
  void Bind(const std::string &endpoint) override;
  void Connect(const std::string &endpoint) override;
  void SetHandler(std::function<void(MessagePtr)> handler) override;
  void Send(MessagePtr message) override;
  void SendWithEnvelope(const std::string &envelope, MessagePtr message) override;
  void SetFilter(const std::string &envelope) override;
  auto Describe() const -> std::string override;

 private:
  SocketType socket_type_;
  std::string trace_socket_info_;
  std::string trace_endpoint_info_;
  std::function<void(MessagePtr)> callback_handler_;
  EndpointObserverT observer_;
  std::deque<MessagePtr> message_queue_;
  DispatchObserverT dispatch_observer_{nullptr};
};

using MocketPtr = std::shared_ptr<Mocket>;

class MocketLogging : public Logging {
 public:
  void LogInfo(const std::string &source, const std::string &msg, uint32_t number) override;
  void LogError(const std::string &source, const std::string &msg, uint32_t number) override;
};

class MocketEventLoop : public EventLoop {
 public:
  // NOLINTNEXTLINE(modernize-pass-by-value)
  explicit MocketEventLoop(const std::string &name) : name(name) {}
  void Run() override {}
  void Exit() override {}
  std::string name;
};

class MocketTimer : public CoreTimer {
 public:
  using Location = std::source_location;
  explicit MocketTimer(uint32_t timer_instance);

  void Dispatch(const std::string &task_name, Location loc = Location::current());
  auto Describe() const -> std::string override;

  // CoreTimer
  void SetHandler(std::function<void(uint32_t, const std::string &)> handler) override;
  auto Request(uint32_t duration_ms, bool periodic, const std::string &task_name) -> uint32_t override;
  void Cancel(uint32_t task_id) override;

  using RequestObserver = std::function<void(uint32_t duration_ms, bool periodic, const std::string &task_name)>;
  using CancelObserver  = std::function<void(const std::string &task_name)>;
  void SetRequestObserver(RequestObserver observer);
  void SetCancelObserver(CancelObserver observer);

  using DispatchObserverT = std::function<void(const MocketTimer &, Location)>;
  void SetDispatchObserver(DispatchObserverT observer);

 private:
  struct Task {
    std::string name;
    uint32_t id   = 0;
    bool periodic = false;
    auto operator<(const Task &other) const -> bool;
  };

  uint32_t timer_instance_ = 0;
  std::function<void(uint32_t, const std::string &)> handler_;
  uint32_t task_id_count_ = 0;
  std::set<Task> tasks_;
  RequestObserver request_observer_{nullptr};
  CancelObserver cancel_observer_{nullptr};
  DispatchObserverT dispatch_observer_{nullptr};
};

using MocketTimerPtr = std::shared_ptr<MocketTimer>;

// Factory
// The MocketFactory constructor inserts itself to be returned when the application
// calls getCoreFactory. The destructor restores normal behaviour.
// MocketFactory should be created/destroyed for each application testcase since it
// has state which should be cleared for each testcase.
class MocketFactory : public CoreFactory {  // NOLINT(*-special-member-functions)
 public:
  MocketFactory();
  ~MocketFactory() override;

  // Mocket:
  auto CreateMocket(SocketType socket_type) -> MocketPtr;
  auto GetMocket(Endpoint method, const std::string &endpoint) -> MocketPtr;
  auto GetMocketTimer(uint32_t timer_instance) -> MocketTimerPtr;

  // CoreFactory
  auto CreateContext() -> std::unique_ptr<Context> override { return {}; }
  auto CreateEventLoop(const std::string &unique_name) -> std::shared_ptr<EventLoop> override {
    return std::make_shared<MocketEventLoop>(unique_name);
  }
  auto CreateCoreSocket(SocketType socket_type, MessageType message_type) -> CoreSocketPtr override;
  auto CreateCoreSocket(EventLoop &event_loop, SocketType socket_type, MessageType message_type)
      -> CoreSocketPtr override;
  auto CreateCoreTimer(EventLoop &event_loop) -> CoreTimerPtr override;
  auto CreatePackagedMessage(uint32_t message_id, size_t size) -> MessagePtr override;
  auto CreateRawMessage(size_t size) -> MessagePtr override;

  auto Describe() const -> std::string;

 private:
  void AddEndpoint(const EndpointInfo &endpoint_info);
  std::map<std::string, std::weak_ptr<Mocket>> bind_endpoints_;
  std::map<std::string, std::weak_ptr<Mocket>> connect_endpoints_;
  uint32_t timer_instance_count_ = 0;
  std::map<uint32_t /*timer_instance*/, std::weak_ptr<MocketTimer>> timers_;
};

inline MocketFactory::MocketFactory() {
  auto factory_generator = [this]() { return this; };
  SetCoreFactoryGenerator(factory_generator);
}

class MocketMessage : public Message {
 public:
  // NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
  MocketMessage(uint32_t message_id, size_t size)
      : message_id_(message_id), message_size_(size), type_(MessageType::PACKAGED) {
    buffer_ = std::make_unique<unsigned char[]>(message_size_);  // NOLINT(*-avoid-c-arrays)
  }

  explicit MocketMessage(size_t size) : message_size_(size), type_(MessageType::RAW) {
    buffer_ = std::make_unique<unsigned char[]>(message_size_);  // NOLINT(*-avoid-c-arrays)
  }

  // Message
  auto Id() const -> uint32_t override { return message_id_; }
  auto Size() const -> size_t override { return message_size_; }
  auto Data() const -> void * override { return buffer_.get(); }
  auto Type() const -> MessageType override { return type_; }

 private:
  uint32_t message_id_ = 0;
  size_t message_size_ = 0;
  MessageType type_;
  std::unique_ptr<unsigned char[]> buffer_;  // NOLINT(*-avoid-c-arrays)
};

inline void MocketFactory::AddEndpoint(const EndpointInfo &endpoint_info) {
  if (Endpoint::BIND == endpoint_info.method) {
    bind_endpoints_[endpoint_info.id] = endpoint_info.mocket;
  } else if (Endpoint::CONNECT == endpoint_info.method) {
    connect_endpoints_[endpoint_info.id] = endpoint_info.mocket;
  } else {
    LOG_ERROR("Invalid endpoint method");
  }
}

inline auto MocketFactory::CreatePackagedMessage(uint32_t message_id, size_t size) -> MessagePtr {
  return std::make_unique<MocketMessage>(message_id, size);
}

inline auto MocketFactory::CreateRawMessage(size_t size) -> MessagePtr { return std::make_unique<MocketMessage>(size); }

inline auto MocketFactory::CreateMocket(SocketType socket_type) -> MocketPtr {
  auto endpoint_observer = [this](const EndpointInfo &endpoint_info) { this->AddEndpoint(endpoint_info); };
  return std::make_shared<Mocket>(socket_type, endpoint_observer);
}

inline auto MocketFactory::GetMocket(Endpoint method, const std::string &endpoint) -> std::shared_ptr<Mocket> {
  if (Endpoint::BIND == method) {
    if (bind_endpoints_.contains(endpoint)) {
      if (auto mocket = bind_endpoints_[endpoint].lock()) {
        return mocket;
      }
    }
  } else if (Endpoint::CONNECT == method) {
    if (connect_endpoints_.contains(endpoint)) {
      if (auto mocket = connect_endpoints_[endpoint].lock()) {
        return mocket;
      }
    }
  } else {
    LOG_ERROR("Invalid endpoint method");
    return nullptr;
  }
  LOG_DEBUG("Mocket not found: {}", endpoint);
  return nullptr;
}

inline auto MocketFactory::GetMocketTimer(uint32_t timer_instance) -> MocketTimerPtr {
  if (!timers_.contains(timer_instance)) {
    LOG_DEBUG("TimerMocket instance not found: {}", timer_instance);
    return {};
  }
  MocketTimerPtr mocket_timer = timers_[timer_instance].lock();
  if (!mocket_timer) {
    LOG_DEBUG("TimerMocket expired for instance: {}", timer_instance);
  }
  return mocket_timer;
}

inline auto MocketFactory::CreateCoreSocket(SocketType socket_type, MessageType /*message_type*/) -> CoreSocketPtr {
  MocketPtr mocket = CreateMocket(socket_type);
  return mocket;
}

inline auto MocketFactory::CreateCoreSocket(EventLoop & /*event_loop*/, SocketType socket_type,
                                            MessageType /*message_type*/) -> CoreSocketPtr {
  MocketPtr mocket = CreateMocket(socket_type);
  return mocket;
}

inline auto MocketFactory::CreateCoreTimer(EventLoop & /*event_loop*/) -> CoreTimerPtr {
  auto timer_instance     = ++timer_instance_count_;
  auto timer_mocket       = std::make_shared<MocketTimer>(timer_instance);
  timers_[timer_instance] = timer_mocket;
  return timer_mocket;
}

inline auto MocketFactory::Describe() const -> std::string {
  std::string description = "{ bind_endpoints: {";
  for (const auto &bind_endpoint : bind_endpoints_) {
    description += (bind_endpoint.first + ",");
  }

  description += "}, connect_endpoints: {";
  for (const auto &connect_endpoint : connect_endpoints_) {
    description += (connect_endpoint.first + ",");
  }

  description += ("}, timers: " + std::to_string(timers_.size()));
  return description;
}

inline MocketFactory::~MocketFactory() { SetCoreFactoryGenerator(std::function<CoreFactory *()>{}); }

// Alternative help functions to get, setup and free MocketFactory
inline auto GetMocketFactory() -> MocketFactory * { return dynamic_cast<MocketFactory *>(GetCoreFactory()); }

inline void SetupMocketFactory() {
  auto mocket_factory = std::make_shared<MocketFactory>();

  auto factory_generator = [mocket_factory]() { return mocket_factory.get(); };
  SetCoreFactoryGenerator(factory_generator);
}

inline void FreeMocketFactory() { SetCoreFactoryGenerator(std::function<CoreFactory *()>{}); }

template <typename Data>
auto Mocket::Receive(Location loc) -> std::optional<Data> {
  static_assert(std::is_trivially_copyable_v<Data> == true);  // NOLINT(readability-simplify-boolean-expr)

  auto expected_id = static_cast<uint32_t>(Data::Metadata::MESSAGE_ID);
  common::logging::Log(fmt::format("{}:{} | Receive {} {:#x}", loc.file_name(), loc.line(), Describe(), expected_id),
                       boost::log::trivial::trace);

  if (message_queue_.empty()) {
    LOG_DEBUG("Mocket::Receive, no messages to receive");
    return {};
  }

  MessagePtr message = nullptr;
  for (auto it = message_queue_.rbegin(); it != message_queue_.rend(); ++it) {
    if ((*it)->Id() == expected_id) {
      message = std::move(*it);
      message_queue_.erase(std::next(it).base());
      break;
    }
  }

  if (!message) {
    LOG_DEBUG("Mocket::receive, message id:{:#x} not found!", expected_id);
    return {};
  }

  auto *source       = message->Data();
  auto *typed_source = static_cast<Data *>(source);

  return *typed_source;
}

inline auto Mocket::ReceiveMessage(Location loc) -> MessagePtr {
  common::logging::Log(fmt::format("{}:{} | ReceiveMessage {}", loc.file_name(), loc.line(), Describe()),
                       boost::log::trivial::trace);

  return ReceiveMessageNoLog(loc);
}

inline auto Mocket::ReceiveMessageNoLog(Location loc) -> MessagePtr {
  if (message_queue_.empty()) {
    common::logging::Log(
        fmt::format("{}:{} | ReceiveMessage, no messages to receive, {}", loc.file_name(), loc.line(), Describe()),
        boost::log::trivial::error);
    return {};
  }
  auto message = std::move(message_queue_.back());
  message_queue_.pop_back();
  return message;
}

inline auto Mocket::Receive(const std::function<bool(const MessagePtr &message)> &fn_match, Location loc)
    -> MessagePtr {
  MessagePtr message = nullptr;
  for (auto it = message_queue_.rbegin(); it != message_queue_.rend(); ++it) {
    if (fn_match(*it)) {
      message = std::move(*it);
      message_queue_.erase(std::next(it).base());

      common::logging::Log(fmt::format("{}:{} | Receive {}", loc.file_name(), loc.line(), Describe()),
                           boost::log::trivial::trace);

      break;
    }
  }

  if (!message) {
    LOG_DEBUG("Mocket::receive, message not found!");
    return {};
  }

  return message;
}

template <typename Data>
void Mocket::Dispatch(Data data, Location loc) {
  static_assert(std::is_trivially_copyable_v<Data> == true);  // NOLINT(readability-simplify-boolean-expr)

  auto message_id = static_cast<uint32_t>(Data::Metadata::MESSAGE_ID);
  common::logging::Log(fmt::format("{}:{} | Dispatch {} {:#x}", loc.file_name(), loc.line(), Describe(), message_id),
                       boost::log::trivial::trace);

  auto message        = GetCoreFactory()->CreatePackagedMessage(message_id, sizeof(data));
  auto *destination_p = message->Data();
  std::memcpy(destination_p, &data, sizeof(data));  // write the struct to the message buffer
  if (!callback_handler_) {
    LOG_ERROR("Mocket::Dispatch, no callback handler set, {}", Describe());
    assert(false);
  }
  callback_handler_(std::move(message));

  if (dispatch_observer_) {
    dispatch_observer_(*this, loc);
  }
}

inline void Mocket::DispatchMessage(MessagePtr message, Location loc) {
  common::logging::Log(
      fmt::format("{}:{} | DispatchMessage {} {:#x}", loc.file_name(), loc.line(), Describe(), message->Id()),
      boost::log::trivial::trace);

  DispatchMessageNoLog(std::move(message), loc);
}

inline void Mocket::DispatchMessageNoLog(MessagePtr message, Location loc) {
  if (!callback_handler_) {
    LOG_ERROR("Mocket::DispatchMessage, no callback handler set, {}", Describe());
    assert(false);
  }
  callback_handler_(std::move(message));

  if (dispatch_observer_) {
    dispatch_observer_(*this, loc);
  }
}

inline void Mocket::SetDispatchObserver(DispatchObserverT observer) { dispatch_observer_ = std::move(observer); }

// NOLINTNEXTLINE(modernize-pass-by-value)
inline Mocket::Mocket(SocketType socket_type, const EndpointObserverT &observer)
    : socket_type_(socket_type), observer_(observer) {
  switch (socket_type) {
    case SocketType::PUB:
      trace_socket_info_ = "PubSocket";
      break;
    case SocketType::SUB:
      trace_socket_info_ = "SubSocket";
      break;
    case SocketType::PAIR:
      trace_socket_info_ = "PairSocket";
      break;
    default:
      LOG_ERROR("Invalid socket type");
  }
}

inline void Mocket::Bind(const std::string &endpoint) {
  trace_endpoint_info_ = endpoint + ":bind";
  const EndpointInfo endpoint_info{Endpoint::BIND, endpoint, weak_from_this()};
  observer_(endpoint_info);
}

inline void Mocket::Connect(const std::string &endpoint) {
  trace_endpoint_info_ = endpoint + ":connect";
  const EndpointInfo endpoint_info{Endpoint::CONNECT, endpoint, weak_from_this()};
  observer_(endpoint_info);
}

inline void Mocket::SetHandler(std::function<void(MessagePtr)> handler) { callback_handler_ = handler; }

inline void Mocket::Send(MessagePtr message) {
  const bool correct_socket_type = ((SocketType::PUB == socket_type_) || (SocketType::PAIR == socket_type_));
  if (!correct_socket_type) {
    LOG_ERROR("send(): incorrect SocketType");
    assert(false);
  }

  if (message->Type() == MessageType::PACKAGED) {
    LOG_TRACE("queueing message {:#x} on {}", message->Id(), Describe());
  } else {
    LOG_TRACE("queueing message on {}", Describe());
  }
  message_queue_.push_front(std::move(message));
}

inline void Mocket::SendWithEnvelope(const std::string & /*envelope*/, MessagePtr message) {
  // Currently not using envelope in the test support
  const bool correct_socket_type = ((SocketType::PUB == socket_type_) || (SocketType::PAIR == socket_type_));
  if (!correct_socket_type) {
    LOG_ERROR("SendWithEnvelope(): incorrect SocketType");
    assert(false);
  }

  if (message->Type() == MessageType::PACKAGED) {
    LOG_TRACE("queueing message {:#x} on {}", message->Id(), Describe());
  } else {
    LOG_TRACE("queueing message on {}", Describe());
  }
  message_queue_.push_front(std::move(message));
}

inline void Mocket::SetFilter(const std::string &envelope) {
  // Currently not using envelope in the test support
}

inline auto Mocket::Describe() const -> std::string { return trace_socket_info_ + " " + trace_endpoint_info_; }

inline void MocketLogging::LogInfo(const std::string &source, const std::string &msg, uint32_t number) {
  LOG_INFO("{} {} {:#x}", source, msg, number);
}
inline void MocketLogging::LogError(const std::string &source, const std::string &msg, uint32_t number) {
  LOG_ERROR("{} {} {:#x}", source, msg, number);
}

inline MocketTimer::MocketTimer(uint32_t timer_instance) : timer_instance_(timer_instance) {}

inline void MocketTimer::SetDispatchObserver(DispatchObserverT observer) { dispatch_observer_ = std::move(observer); }

// For single-shot timeouts the task could be removed from the map at dispatch.
// If there are multiple task with the same task_name, the one with the lowest task_id
// will be dispatched (and possibly removed).
inline void MocketTimer::Dispatch(const std::string &task_name, Location loc) {
  common::logging::Log(fmt::format("{}:{} | Dispatch, timer_instance={}, task_name={}", loc.file_name(), loc.line(),
                                   timer_instance_, task_name),
                       boost::log::trivial::trace);

  // Finds the right task name and gets the one with the lowest id (due to the definition of operator<)
  auto task =
      std::find_if(tasks_.begin(), tasks_.end(), [task_name](const auto &task) { return task.name == task_name; });

  if (task == tasks_.end()) {
    LOG_ERROR("MocketTimer::Dispatch: could not find taskname:{}", task_name);
    return;
  }

  // Taking a copy of id and name to avoid reentrancy problems if erasing task
  auto id   = task->id;
  auto name = task->name;

  if (!task->periodic) {
    tasks_.erase(task);
  }

  handler_(id, name);

  if (dispatch_observer_) {
    dispatch_observer_(*this, loc);
  }
}

inline auto MocketTimer::Request(uint32_t duration_ms, bool periodic, const std::string &task_name) -> uint32_t {
  auto task_id = ++task_id_count_;
  tasks_.insert({task_name, task_id, periodic});
  if (request_observer_) {
    request_observer_(duration_ms, periodic, task_name);
  }
  return task_id;
}

inline void MocketTimer::Cancel(uint32_t task_id) {
  auto task = find_if(tasks_.begin(), tasks_.end(), [task_id](const auto &task) { return task.id == task_id; });
  if ((task != tasks_.end()) && cancel_observer_) {
    cancel_observer_(task->name);
  }
  std::erase_if(tasks_, [task_id](const auto &task) { return task.id == task_id; });
}

inline void MocketTimer::SetRequestObserver(RequestObserver observer) { request_observer_ = std::move(observer); }

inline void MocketTimer::SetCancelObserver(CancelObserver observer) { cancel_observer_ = std::move(observer); }

inline void MocketTimer::SetHandler(std::function<void(uint32_t, const std::string &)> handler) { handler_ = handler; }

inline auto MocketTimer::Task::operator<(const Task &other) const -> bool {
  return (name < other.name) || ((name == other.name) && (id < other.id));
}

inline auto MocketTimer::Describe() const -> std::string {
  std::string desc = "{";
  for (const auto &task : tasks_) {
    std::string periodic_or_once = (task.periodic) ? "periodic" : "once";
    desc += "{id=" + std::to_string(task.id) + " , name=" + task.name + ", " + periodic_or_once + "},";
  }
  desc += "}";

  return desc;
}

struct TimerS {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x10000 };
  uint32_t task_id;
};

}  // namespace zevs
