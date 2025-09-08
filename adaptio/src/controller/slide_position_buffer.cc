#include "slide_position_buffer.h"

#include <cstdint>

using controller::SlidePositionBufferImpl;

namespace {
const uint32_t MAX_BUFFER_SIZE = 100;

}  // namespace

SlidePositionBufferImpl::SlidePositionBufferImpl() : buffer_(MAX_BUFFER_SIZE) {}

auto SlidePositionBufferImpl::StorePosition(double position, uint64_t time_stamp) -> bool {
  SlidePositionEntry entry{};
  entry.position   = position;
  entry.time_stamp = time_stamp;

  buffer_.push_front(entry);
  return true;
}

auto SlidePositionBufferImpl::GetPosition(uint64_t time_stamp) -> double {
  int i;

  // Check special case when all items in buffer are older than the requested
  // time stamp.
  if (buffer_[0].time_stamp <= time_stamp) {
    return buffer_[0].position;
  }

  // Get closest before and closest after
  // Perform a linear interpolation between them.
  for (i = 0; i < buffer_.size() && buffer_[i].time_stamp > time_stamp; i++);

  // Check special case when all items in buffer are newer than the requested
  // time stamp
  if (i == buffer_.size()) {
    return buffer_[i - 1].position;
  }

  auto t0 = buffer_[i].time_stamp;
  auto t1 = buffer_[i - 1].time_stamp;
  auto p0 = buffer_[i].position;
  auto p1 = buffer_[i - 1].position;

  // (t-t0)/(t1-t0) = (p - p0)/(p1-p0)
  auto a1 = static_cast<double>(time_stamp - t0);
  auto a2 = static_cast<double>(t1 - t0);
  auto a3 = p1 - p0;
  auto p  = p0 + a1 * a3 / a2;

  return p;
}

auto SlidePositionBufferImpl::Capacity() -> uint64_t { return buffer_.capacity(); }
