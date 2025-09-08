#include "timer_backend.h"

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "../zevs_core.h"
#include "common/logging/application_log.h"
#include "coresocket_impl.h"

namespace zevs {

const std::string ADAPTIO_TIMER = "adaptio_timer";

struct TimerS {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x10000 };
  uint32_t task_id;
};

class TimerSubscriberImpl : public TimerSubscriber {
 public:
  TimerSubscriberImpl(const std::string& client_endpoint);
  void Init() override;  // To be called in the timer thread
  void Send(uint32_t task_id) override;

 private:
  std::string client_endpoint_;
  CoreSocketPtr socket_;
};

TimerSubscriberImpl::TimerSubscriberImpl(const std::string& client_endpoint) : client_endpoint_(client_endpoint) {}

void TimerSubscriberImpl::Init() {
  // pub socket is used, a pair socket could block on send if the
  // receiving socket is deleted.
  socket_          = GetCoreFactory()->CreateCoreSocket(SocketType::PUB, MessageType::PACKAGED);
  auto socket_impl = std::dynamic_pointer_cast<CoreSocketImpl>(socket_);
  socket_impl->SuppressEventTrace();  // to much trace from timeouts
  socket_->Bind(client_endpoint_);
}

void TimerSubscriberImpl::Send(uint32_t task_id) {
  auto message_id = static_cast<uint32_t>(TimerS::Metadata::MESSAGE_ID);
  TimerS msg{task_id};
  auto message = GetCoreFactory()->CreatePackagedMessage(message_id, sizeof(msg));
  memcpy(message->Data(), &msg, sizeof(msg));
  socket_->SendWithEnvelope(ADAPTIO_TIMER, std::move(message));
}

TimerBackend::TimerBackend(const std::string& timer_endpoint, CoreSocketPtr core_socket)
    : timer_endpoint_(timer_endpoint), core_socket_(core_socket) {
  // prepare the timeout subscription socket
  auto message_handler = [this](MessagePtr msg) { this->OnTimeoutMessage(std::move(msg)); };
  core_socket_->SetHandler(message_handler);
  core_socket_->SetFilter("");

  auto core_socket_impl = std::dynamic_pointer_cast<CoreSocketImpl>(core_socket_);
  core_socket_impl->SuppressEventTrace();  // to much trace from timeouts
  core_socket_->Connect(timer_endpoint_);

  // start the timer thread (will be one such thread for each eventloop)
  auto timer_subscriber = std::make_unique<TimerSubscriberImpl>(timer_endpoint_);
  timer_thread_         = std::make_unique<TimerThread>(std::move(timer_subscriber));
  thread_               = std::thread(&TimerThread::Run, timer_thread_.get());
}

TimerBackend::~TimerBackend() { ExitTimerThread(); }

void TimerBackend::ExitTimerThread() {
  if (thread_.joinable()) {
    // see note in timer_thread.h regarding the order below
    timer_thread_->Exit();
    thread_.join();
    timer_thread_.reset();
  }
}

void TimerBackend::OnTimeoutMessage(MessagePtr msg) {
  if (msg->Id() != static_cast<uint32_t>(TimerS::Metadata::MESSAGE_ID)) {
    LOG_ERROR("TimerBackend::OnTimeoutMessage(), invalid message id: {}", msg->Id());
    return;
  }

  auto* data   = static_cast<TimerS*>(msg->Data());
  auto task_id = data->task_id;

  if (!handlers_.contains(task_id)) {
    // This can happen if a thread has requested periodic timeouts and does
    // processing at each timeout with a duration longer than the timeout period.
    // Timeout messages then queue up. If the timer task is later cancelled
    // the queued messages will be unhandled. This scenario could be handled
    // by having a circular buffer of cancelled task identifiers in case this is
    // considered to be a significant problem.

    LOG_INFO("TimeoutMessage with unknown task_id: {} (queued up)", task_id);
    return;
  }

  // Taking a copy of the handler since the callback can be reentrant and
  // erase the handler from the map.
  auto handler = handlers_[task_id];
  auto done    = handler(task_id);

  if (done) {
    // This was a single timerout or callback expired
    handlers_.erase(task_id);
  }
}

auto TimerBackend::Request(OnTimeoutFunc on_timeout, uint32_t duration_ms, bool periodic) -> uint32_t {
  DurationMilliSec duration{duration_ms};
  auto task_id = timer_thread_->Request(duration, periodic);
  handlers_.insert({task_id, on_timeout});
  return task_id;
}

void TimerBackend::Cancel(uint32_t task_id) {
  timer_thread_->Cancel(task_id);
  handlers_.erase(task_id);
}

}  // namespace zevs
