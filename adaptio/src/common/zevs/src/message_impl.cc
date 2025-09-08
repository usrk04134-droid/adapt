#include "message_impl.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <zmq.hpp>

#include "../zevs_core.h"
#include "common/logging/application_log.h"

namespace zevs {

// Packet is only used for packaged messages
struct Packet {
  uint32_t message_id{0};

  // size 0 gives compile error with current settings. Size=1 works fine since the size of Packet
  // does not affect allocation anyway.
  char message[1];
};

PackagedMessage::PackagedMessage(uint32_t id, size_t size) : id_(id), message_size_{size} {
  auto zmq_size = sizeof(uint32_t) + message_size_;
  zmq_message_  = std::make_unique<zmq::message_t>(zmq_size);

  auto *packet       = static_cast<Packet *>(zmq_message_->data());
  packet->message_id = id_;
}

PackagedMessage::PackagedMessage(zmq::message_t &&from_zmq_message) {
  auto zmq_size = from_zmq_message.size();
  if (zmq_size < sizeof(uint32_t)) {
    LOG_ERROR("PackagedMessage(), size to small");
  }

  message_size_ = zmq_size - sizeof(uint32_t);
  auto *packet  = static_cast<Packet *>(from_zmq_message.data());
  id_           = packet->message_id;
  zmq_message_  = std::make_unique<zmq::message_t>(std::move(from_zmq_message));
}

auto PackagedMessage::Id() const -> uint32_t { return id_; }

auto PackagedMessage::Size() const -> size_t { return message_size_; }

auto PackagedMessage::Data() const -> void * {
  auto *packet = static_cast<Packet *>(zmq_message_->data());
  return packet->message;
}

auto PackagedMessage::Type() const -> MessageType { return MessageType::PACKAGED; }

auto PackagedMessage::GetZmqMessage() const -> zmq::message_t * { return zmq_message_.get(); }

//
//************* RawMessage ********************

RawMessage::RawMessage(size_t size) : message_size_{size} {
  auto zmq_size = message_size_;
  zmq_message_  = std::make_unique<zmq::message_t>(zmq_size);
}

RawMessage::RawMessage(zmq::message_t &&from_zmq_message) {
  auto zmq_size = from_zmq_message.size();
  message_size_ = zmq_size;
  zmq_message_  = std::make_unique<zmq::message_t>(std::move(from_zmq_message));
}

auto RawMessage::Id() const -> uint32_t {
  LOG_ERROR("id not available for RawMessage");
  return 0;
}

auto RawMessage::Size() const -> size_t { return message_size_; }

auto RawMessage::Data() const -> void * { return zmq_message_->data(); }

auto RawMessage::Type() const -> MessageType { return MessageType::RAW; }

auto RawMessage::GetZmqMessage() const -> zmq::message_t * { return zmq_message_.get(); }

}  // namespace zevs
