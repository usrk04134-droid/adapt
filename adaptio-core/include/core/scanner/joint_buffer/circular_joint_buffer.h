#pragma once

#include <boost/circular_buffer.hpp>
#include <memory>
#include <optional>

#include "core/image/camera_model.h"
#include "core/scanner/joint_buffer/joint_buffer.h"
#include "core/scanner/joint_model.h"

namespace core::scanner {

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

}  // namespace core::scanner
