#include "scanner/slice_provider/circular_joint_buffer.h"

#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

namespace scanner::slice_provider {

CircularJointBuffer::CircularJointBuffer() : m_buffer(7) {}

void CircularJointBuffer::AddSlice(const JointSlice& slice) { m_buffer.push_back(slice); }

auto CircularJointBuffer::GetSlice() const -> std::optional<JointSlice> {
  // Return last, for now.
  if (!m_buffer.empty()) {
    return (m_buffer.back());
  }

  return std::nullopt;
}

auto CircularJointBuffer::GetLatestTimestamp() const -> std::optional<Timestamp> {
  if (!m_buffer.empty()) {
    return m_buffer.back().timestamp;
  }

  return std::nullopt;
}

auto CircularJointBuffer::GetRecentSlices(long time_period_ms) const -> std::vector<JointSlice*> {
  std::vector<JointSlice*> v;
  auto cutoff = std::chrono::high_resolution_clock::now() - std::chrono::milliseconds(time_period_ms);
  for (auto it = m_buffer.rbegin(); it != m_buffer.rend() && it->timestamp > cutoff; ++it) {
    // Ok to return a vector of pointers to elements in m_buffer since the mutex lock is on when returned vector is used
    v.push_back(const_cast<JointSlice*>(&(*it)));
  }

  return v;
}

auto CircularJointBuffer::GetNumberOfSlices() const -> uint64_t { return m_buffer.size(); }

void CircularJointBuffer::Reset() { m_buffer.clear(); }

}  // namespace scanner::slice_provider
