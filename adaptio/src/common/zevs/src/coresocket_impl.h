#pragma once

#include <functional>
#include <memory>
#include <string>

#include "../zevs_core.h"
#include "zmq.hpp"

namespace zevs {

class CoreSocketImpl : public CoreSocket {
 public:
  CoreSocketImpl(SocketType socket_type, MessageType message_type, std::unique_ptr<zmq::socket_t> socket);

  // Socket
  void Bind(const std::string &endpoint) override;
  void Connect(const std::string &endpoint) override;
  void SetHandler(std::function<void(MessagePtr)> handler) override;
  void Send(MessagePtr message) override;
  void SendWithEnvelope(const std::string &envelope, MessagePtr message) override;
  void SetFilter(const std::string &envelope) override;
  auto Describe() const -> std::string override;

  // Called from Eventloop
  void OnEvent();

  // Trace
  void SuppressEventTrace();

 private:
  SocketType socket_type_;
  MessageType message_type_;
  std::unique_ptr<zmq::socket_t> socket_;
  std::function<void(MessagePtr)> message_handler_;
  std::string trace_socket_info_;
  std::string trace_description_;
  bool event_trace_ = true;
};

}  // namespace zevs
