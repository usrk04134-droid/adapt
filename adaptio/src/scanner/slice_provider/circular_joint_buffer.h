#pragma once

#include <boost/circular_buffer.hpp>
#include <memory>
#include <optional>

#include "scanner/image/camera_model.h"
#include "scanner/joint_model/joint_model.h"
#include "scanner/slice_provider/slice_provider.h"

namespace scanner::slice_provider {

class CircularJointBuffer {
 public:
  CircularJointBuffer();

  void AddSlice(const JointSlice& slice);

  [[nodiscard]] auto GetSlice() const -> std::optional<JointSlice>;

  [[nodiscard]] auto GetLatestTimestamp() const -> std::optional<Timestamp>;

  [[nodiscard]] auto GetRecentSlices(long) const -> std::vector<JointSlice*>;

  [[nodiscard]] auto GetNumberOfSlices() const -> uint64_t;

  void Reset();

 private:
  boost::circular_buffer<JointSlice> m_buffer;
};

}  // namespace scanner::slice_provider
