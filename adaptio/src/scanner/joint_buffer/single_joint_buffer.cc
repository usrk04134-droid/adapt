#include "scanner/joint_buffer/single_joint_buffer.h"

#include <cstdint>
#include <optional>
#include <vector>

#include "scanner/joint_buffer/joint_buffer.h"

namespace scanner::joint_buffer {
SingleJointBuffer::SingleJointBuffer() {}

void SingleJointBuffer::AddSlice(const JointSlice& slice) { slice_ = slice; }

auto SingleJointBuffer::GetSlice() const -> std::optional<JointSlice> { return slice_; }

auto SingleJointBuffer::GetLatestTimestamp() const -> std::optional<Timestamp> { return slice_->timestamp; }

auto SingleJointBuffer::GetNumberOfSlices() const -> uint64_t {
  if (slice_.has_value()) {
    return 1;
  }

  return 0;
}

auto SingleJointBuffer::GetRecentSlices(long) const -> std::vector<JointSlice*> {
  std::vector<JointSlice*> v;
  if (slice_.has_value()) {
    auto slice = slice_.value();
    v.push_back(&slice);
  }

  return v;
}

void SingleJointBuffer::Reset() { slice_ = std::nullopt; }
}  // namespace scanner::joint_buffer
