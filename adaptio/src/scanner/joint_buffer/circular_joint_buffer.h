#pragma once

#include <boost/circular_buffer.hpp>
#include <memory>
#include <optional>

#include "scanner/image/camera_model.h"
#include "scanner/joint_buffer/joint_buffer.h"
#include "scanner/joint_model/joint_model.h"

namespace scanner::joint_buffer {

class CircularJointBuffer : public JointBuffer {
 public:
  CircularJointBuffer();

  void AddSlice(const JointSlice& slice) override;

  [[nodiscard]] auto GetSlice() const -> std::optional<JointSlice> override;

  [[nodiscard]] auto GetLatestTimestamp() const -> std::optional<Timestamp> override;

  [[nodiscard]] auto GetRecentSlices(long) const -> std::vector<JointSlice*> override;

  [[nodiscard]] auto GetNumberOfSlices() const -> uint64_t override;

  void Reset() override;

 private:
  boost::circular_buffer<JointSlice> m_buffer;
};

}  // namespace scanner::joint_buffer
