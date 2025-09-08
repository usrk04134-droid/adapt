#pragma once

#include <boost/circular_buffer.hpp>
#include <chrono>
#include <memory>
#include <optional>

#include "common/clock_functions.h"
#include "scanner/joint_buffer/joint_buffer.h"
#include "scanner/joint_model/joint_model.h"
#include "scanner/slice_provider/slice_provider.h"

namespace scanner::slice_provider {

auto const RECENT_SLICES_MS = 400;

class SliceProviderImpl : public SliceProvider {
 public:
  explicit SliceProviderImpl(joint_buffer::JointBufferPtr joint_buffer,
                             clock_functions::SteadyClockNowFunc steady_clock_now_func);

  void AddSlice(const scanner::joint_buffer::JointSlice& slice) override;
  auto GetSlice() -> std::optional<joint_model::JointProfile> override;
  auto GetTrackingSlice() -> std::optional<std::tuple<joint_tracking::JointSlice, uint64_t, double>> override;
  auto SliceDegraded() -> bool override { return slice_degraded_; };
  void Reset() override;

 private:
  auto GetLatestTimestamp() const -> std::optional<Timestamp> { return joint_buffer_->GetLatestTimestamp(); };
  auto MedianOfRecentSlices() -> std::optional<joint_buffer::JointSlice>;
  auto GetConfidence(joint_buffer::JointSlice slice) -> joint_tracking::SliceConfidence;

  joint_buffer::JointBufferPtr joint_buffer_;
  joint_tracking::JointSlice latest_slice_;
  std::optional<std::chrono::time_point<std::chrono::steady_clock>> last_sent_ts_;
  bool slice_degraded_{false};
  clock_functions::SteadyClockNowFunc steady_clock_now_func_;
  double last_area_{0.0};
};

}  // namespace scanner::slice_provider
