#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "../zevs_core.h"
#include "zmq.hpp"

namespace zevs {

// The zmq_message buffer will contain one of the following:
// For raw message: only user data, such as a json string from another process
// For packaged message: message id plus a memcopy of a trivially copyable struct.
// size() is the size of the user data or the size of the pointer.

class PackagedMessage : public Message {
 public:
  PackagedMessage(uint32_t id, size_t size);               // For send
  explicit PackagedMessage(zmq::message_t&& zmq_message);  // For receive

  // Message
  auto Id() const -> uint32_t override;
  auto Size() const -> size_t override;
  auto Data() const -> void* override;
  auto Type() const -> MessageType override;

  // Impl
  auto GetZmqMessage() const -> zmq::message_t*;

 private:
  uint32_t id_         = 0;
  size_t message_size_ = 0;
  std::unique_ptr<zmq::message_t> zmq_message_;
};

class RawMessage : public Message {
 public:
  explicit RawMessage(size_t size);                   // For send
  explicit RawMessage(zmq::message_t&& zmq_message);  // For receive

  // Message
  auto Id() const -> uint32_t override;
  auto Size() const -> size_t override;
  auto Data() const -> void* override;
  auto Type() const -> MessageType override;

  // Impl
  auto GetZmqMessage() const -> zmq::message_t*;

 private:
  size_t message_size_ = 0;
  std::unique_ptr<zmq::message_t> zmq_message_;
};

}  // namespace zevs
