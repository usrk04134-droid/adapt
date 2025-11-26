#include "scanner/slice_provider/circular_joint_buffer.h"

#include <chrono>
#include <utility>

namespace scanner::slice_provider {

namespace {
constexpr size_t kDefaultBufferCapacity = 7;
}  // namespace

CircularJointBuffer::CircularJointBuffer() : buffer_(kDefaultBufferCapacity) {}

CircularJointBuffer::CircularJointBuffer(size_t capacity) : buffer_(capacity) {}

void CircularJointBuffer::AddSlice(const JointSlice& slice) { buffer_.push_back(slice); }

auto CircularJointBuffer::GetSlice() const -> std::optional<JointSlice> {
  if (!buffer_.empty()) {
    return buffer_.back();
  }

  return std::nullopt;
}

auto CircularJointBuffer::GetLatestTimestamp() const -> std::optional<Timestamp> {
  if (!buffer_.empty()) {
    return buffer_.back().timestamp;
  }

  return std::nullopt;
}

auto CircularJointBuffer::GetRecentSlices(long time_period_ms) const -> std::vector<JointSlice*> {
  std::vector<JointSlice*> recent_slices;
  auto cutoff = std::chrono::high_resolution_clock::now() - std::chrono::milliseconds(time_period_ms);

  for (auto it = buffer_.rbegin(); it != buffer_.rend() && it->timestamp > cutoff; ++it) {
    // Constellation relying on callers synchronizing access to the buffer.
    recent_slices.push_back(const_cast<JointSlice*>(&(*it)));
  }

  return recent_slices;
}

auto CircularJointBuffer::GetNumberOfSlices() const -> uint64_t { return buffer_.size(); }

void CircularJointBuffer::Reset() { buffer_.clear(); }

}  // namespace scanner::slice_provider
