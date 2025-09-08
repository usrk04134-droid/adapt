#include "coresocket_impl.h"

#include <fmt/core.h>

#include <boost/log/trivial.hpp>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iterator>
#include <memory>
#include <source_location>
#include <string>
#include <utility>
#include <vector>
#include <zmq.hpp>
#include <zmq_addon.hpp>

#include "../zevs_core.h"
#include "common/logging/application_log.h"
#include "message_impl.h"

namespace zevs {
using boost::log::sources::severity_logger;
using boost::log::trivial::severity_level;
using Location = std::source_location;

CoreSocketImpl::CoreSocketImpl(SocketType socket_type, MessageType message_type, std::unique_ptr<zmq::socket_t> socket)
    : socket_type_(socket_type), message_type_(message_type), socket_(std::move(socket)) {
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

void CoreSocketImpl::Bind(const std::string &endpoint) {
  // Allowing sub socket to bind for now
  bool correct_socket_type =
      ((SocketType::PUB == socket_type_) || (SocketType::SUB == socket_type_) || (SocketType::PAIR == socket_type_));
  if (!correct_socket_type) {
    LOG_ERROR("bind(): incorrect SocketType");
    return;
  }
  trace_description_ = trace_socket_info_ + " " + endpoint + ":bind";
  socket_->bind(endpoint);
}

void CoreSocketImpl::Connect(const std::string &endpoint) {
  // Allowing pub socket to connect for now
  bool correct_socket_type =
      ((SocketType::SUB == socket_type_) || (SocketType::PUB == socket_type_) || (SocketType::PAIR == socket_type_));
  if (!correct_socket_type) {
    LOG_ERROR("connect(): incorrect SocketType");
    return;
  }
  trace_description_ = trace_socket_info_ + " " + endpoint + ":connect";
  socket_->connect(endpoint);
}

void CoreSocketImpl::SetHandler(std::function<void(MessagePtr)> handler) {
  bool correct_socket_type = ((SocketType::SUB == socket_type_) || (SocketType::PAIR == socket_type_));
  if (!correct_socket_type) {
    LOG_ERROR("setHandler(): incorrect SocketType, {}", Describe());
    return;
  }
  message_handler_ = handler;
}

// BOOST_LOG_SEV is used together with the below help function in this file.
// The reason is that the macro LOG_TRACE will create the message string
// and perform fmt::format even if the severity level is below the currently
// set loglevel. When using BOOST_LOG_SEV the LogString function will not be
// called unless the log is actually active.
// This is only applied for frequent logs (send/receive for packaged message)
std::string LogString(const std::string &obj, const std::string &operation, uint32_t message_id,
                      Location loc = Location::current()) {
  auto log_string = std::string(loc.file_name())
                        .append(":")
                        .append(std::to_string(loc.line()))
                        .append(" | ")
                        .append(fmt::format("{}: {}: {:#x}", obj, operation, message_id));
  return log_string;
}

void CoreSocketImpl::Send(MessagePtr message) {
  if (!message) {
    LOG_ERROR("send(): empty message, {}", Describe());
    return;
  }

  bool correct_socket_type = (SocketType::PAIR == socket_type_);
  if (!correct_socket_type) {
    LOG_ERROR("send(): incorrect SocketType, {}", Describe());
    return;
  }
  if (message_type_ != message->Type()) {
    LOG_ERROR("send(): incorrect MessageType, {}", Describe());
    return;
  }

  zmq::message_t *zmq_msg = nullptr;
  if (MessageType::PACKAGED == message->Type()) {
    auto *message_impl = dynamic_cast<PackagedMessage *>(message.get());
    zmq_msg            = message_impl->GetZmqMessage();
    // Zevs send/receive traces are disabled as long as:
    // A. Messages are used for copying data between threads (disparate structs with a set of misc bools) instead
    // of beeing distinct events such as start, stop etc. The signal number gives little info currently.
    //
    // B. Data is not serialized using protobuf or similar which would allow the data content to be traced.
    //
    // if (event_trace_) {
    //   severity_logger<severity_level> logger;
    //   BOOST_LOG_SEV(logger, boost::log::trivial::trace) << LogString(trace_description_, "send", message_impl->Id());
    // }
  } else {
    auto *message_impl = dynamic_cast<RawMessage *>(message.get());
    zmq_msg            = message_impl->GetZmqMessage();
    // if (event_trace_) {
    //   LOG_TRACE("{}: send: raw message", trace_description_);
    // }
  }
  socket_->send(*zmq_msg, zmq::send_flags::none);
}

void CoreSocketImpl::SendWithEnvelope(const std::string &envelope, MessagePtr message) {
  if (envelope.empty()) {
    LOG_ERROR("SendWithEnvelope(): empty envelope, {}", Describe());
    return;
  }
  if (!message) {
    LOG_ERROR("SendWithEnvelope(): empty message, {}", Describe());
    return;
  }

  bool correct_socket_type = (SocketType::PUB == socket_type_);
  if (!correct_socket_type) {
    LOG_ERROR("SendWithEnvelope(): incorrect SocketType, {}", Describe());
    return;
  }
  if (message_type_ != message->Type()) {
    LOG_ERROR("SendWithEnvelope(): incorrect MessageType, {}", Describe());
    return;
  }

  // Send envelope/topic with sndmore flag
  zmq::message_t envelope_msg{envelope.size()};
  std::memcpy(envelope_msg.data(), envelope.c_str(), envelope.size());
  socket_->send(envelope_msg, zmq::send_flags::sndmore);

  zmq::message_t *zmq_msg = nullptr;
  if (MessageType::PACKAGED == message->Type()) {
    auto *message_impl = dynamic_cast<PackagedMessage *>(message.get());
    zmq_msg            = message_impl->GetZmqMessage();
    // See comment above regarding zevs send/receive traces
    //
    // if (event_trace_) {
    //   severity_logger<severity_level> logger;
    //   BOOST_LOG_SEV(logger, boost::log::trivial::trace)
    //       << LogString(trace_description_, "send, envelope=" + envelope, message_impl->Id());
    // }
  } else {
    auto *message_impl = dynamic_cast<RawMessage *>(message.get());
    zmq_msg            = message_impl->GetZmqMessage();
    // if (event_trace_) {
    //   LOG_TRACE("{}: send, envelope={}: raw message", trace_description_, envelope);
    // }
  }
  socket_->send(*zmq_msg, zmq::send_flags::none);
}

void CoreSocketImpl::SetFilter(const std::string &envelope) {
  bool correct_socket_type = (SocketType::SUB == socket_type_);
  if (!correct_socket_type) {
    LOG_ERROR("SetFilter(): incorrect SocketType, {}", Describe());
    return;
  }

  socket_->set(zmq::sockopt::subscribe, envelope.c_str());
}

auto CoreSocketImpl::Describe() const -> std::string { return trace_description_; }

void CoreSocketImpl::OnEvent() {
  if (!message_handler_) {
    LOG_ERROR("No message handler, {}", Describe());
    return;
  }

  zmq::message_t zmq_msg;
  if (SocketType::SUB == socket_type_) {
    // Should receive an envelope and a message, otherwise it is an error
    std::vector<zmq::message_t> recv_msgs;
    zmq::recv_result_t result = zmq::recv_multipart(*socket_, std::back_inserter(recv_msgs));
    if (!result) {
      LOG_ERROR("Socket recv_multipart error, {}", Describe());
      return;
    }
    if (*result != 2) {
      LOG_ERROR("Socket recv_multipart, incorrect number of message parts, {}", Describe());
      return;
    }

    zmq_msg = std::move(recv_msgs.back());

  } else {  // not SUB socket
    auto res = socket_->recv(zmq_msg, zmq::recv_flags::none);
    if (!res) {
      LOG_ERROR("Socket receive error, {}", Describe());
      return;
    }
  }

  if (MessageType::PACKAGED == message_type_) {
    auto message_impl = std::make_unique<PackagedMessage>(std::move(zmq_msg));
    // See comment above regarding zevs send/receive traces
    //
    // if (event_trace_) {
    //   severity_logger<severity_level> logger;
    //   BOOST_LOG_SEV(logger, boost::log::trivial::trace) << LogString(trace_description_, "receive",
    //   message_impl->Id());
    // }
    message_handler_(std::move(message_impl));
  } else {
    auto message_impl = std::make_unique<RawMessage>(std::move(zmq_msg));
    // if (event_trace_) {
    //   LOG_TRACE("{}: receive: raw message", trace_description_);
    // }
    message_handler_(std::move(message_impl));
  }
}

void CoreSocketImpl::SuppressEventTrace() { event_trace_ = false; }

}  // namespace zevs
