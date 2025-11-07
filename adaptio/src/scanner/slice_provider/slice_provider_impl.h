#pragma once

#include <boost/circular_buffer.hpp>
#include <chrono>
#include <optional>

#include "common/clock_functions.h"
#include "common/groove/groove.h"
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
  auto GetTrackingSlice()
      -> std::optional<std::tuple<common::Groove, InterpolatedLine, SliceConfidence, uint64_t>> override;
  auto SliceDegraded() -> bool override { return slice_degraded_; };
  void Reset() override;
  auto GetLatestSlice() -> std::optional<scanner::joint_buffer::JointSlice> { return joint_buffer_->GetSlice(); };

 private:
  auto GetLatestTimestamp() const -> std::optional<Timestamp> { return joint_buffer_->GetLatestTimestamp(); };
  auto MedianOfRecentSlices() -> std::optional<joint_buffer::JointSlice>;
  auto GetConfidence(joint_buffer::JointSlice slice) -> SliceConfidence;

  joint_buffer::JointBufferPtr joint_buffer_;
  std::tuple<common::Groove, InterpolatedLine, SliceConfidence> latest_slice_;
  std::optional<std::chrono::time_point<std::chrono::steady_clock>> last_sent_ts_;
  bool slice_degraded_{false};
  clock_functions::SteadyClockNowFunc steady_clock_now_func_;
};

}  // namespace scanner::slice_provider
