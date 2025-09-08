#include "weld_axis_buffer.h"

#include <cstdint>

using controller::WeldAxisBufferImpl;

namespace {
const uint32_t MAX_BUFFER_SIZE = 100;

uint64_t Diff64(uint64_t value1, uint64_t value2) {
  if (value1 > value2) {
    return value1 - value2;
  }

  return value2 - value1;
}
}  // namespace

WeldAxisBufferImpl::WeldAxisBufferImpl() : buffer_(MAX_BUFFER_SIZE), turn_(0) {}

auto WeldAxisBufferImpl::StorePosition(float position, float velocity, uint64_t time_stamp) -> bool {
  PositionElement element{};
  element.weld_axis.position     = position;
  element.weld_axis.velocity     = velocity;
  element.weld_axis.turn_counter = 0;
  element.time_stamp             = time_stamp;

  if (buffer_.empty()) {
    buffer_.push_front(element);
    return true;
  }

  if (buffer_[0].weld_axis.position == position) {
    // Position not updated nothing to do
    return true;
  }

  // Check if position is wrapped i.e new turn
  if (position < buffer_[0].weld_axis.position) {
    element.weld_axis.turn_counter = ++turn_;
  }

  buffer_.push_front(element);
  return true;
}

auto WeldAxisBufferImpl::GetPosition(uint64_t time_stamp) -> WeldAxisPosition {
  auto min_diff = UINT64_MAX;
  auto index    = 0;

  for (int i = 0; i < buffer_.size(); i++) {
    auto diff = Diff64(buffer_[i].time_stamp, time_stamp);

    if (diff > min_diff) {
      // No need to search more since time stamps are in sequential order
      break;
    }

    min_diff = diff;
    index    = i;
  }

  return buffer_[index].weld_axis;
}

auto WeldAxisBufferImpl::Capacity() -> uint64_t { return buffer_.capacity(); }
