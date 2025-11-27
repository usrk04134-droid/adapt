#pragma once

#include <boost/circular_buffer.hpp>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "scanner/slice_provider/slice_provider.h"

namespace scanner::slice_provider {

class CircularJointBuffer {
 public:
  CircularJointBuffer();
  explicit CircularJointBuffer(size_t capacity);

  void AddSlice(const JointSlice& slice);

  [[nodiscard]] auto GetSlice() const -> std::optional<JointSlice>;

  [[nodiscard]] auto GetLatestTimestamp() const -> std::optional<Timestamp>;

  [[nodiscard]] auto GetRecentSlices(long time_period_ms) const -> std::vector<JointSlice*>;

  [[nodiscard]] auto GetNumberOfSlices() const -> uint64_t;

  void Reset();

 private:
  boost::circular_buffer<JointSlice> buffer_;
};

}  // namespace scanner::slice_provider
