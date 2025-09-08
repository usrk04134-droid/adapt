#include "scanner/slice_provider/slice_provider_impl.h"

#include <math.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <iterator>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

#include "common/clock_functions.h"
#include "common/logging/application_log.h"
#include "scanner/joint_buffer/joint_buffer.h"
#include "scanner/joint_model/joint_model.h"
#include "scanner/joint_tracking/joint_slice.h"
#include "scanner/slice_provider/slice_provider.h"

namespace scanner::slice_provider {

using joint_model::HIGH_CONFIDENCE_WALL_HEIGHT;
using joint_model::MEDIUM_CONFIDENCE_WALL_HEIGHT;
using joint_tracking::SliceConfidence;
using std::chrono::high_resolution_clock;

const std::chrono::milliseconds NO_SLICE_DATA_TMO_MS(3000);
const uint32_t MIN_SLICES_FOR_MEDIAN = 3;

SliceProviderImpl::SliceProviderImpl(joint_buffer::JointBufferPtr joint_buffer,
                                     clock_functions::SteadyClockNowFunc steady_clock_now_func)
    : joint_buffer_(std::move(joint_buffer)), steady_clock_now_func_(steady_clock_now_func) {
  std::array<joint_tracking::Coord, 7> coords = {};
  latest_slice_                               = joint_tracking::JointSlice(coords, joint_tracking::SliceConfidence::NO);
}

void SliceProviderImpl::AddSlice(const scanner::joint_buffer::JointSlice& slice) {
  // If this is older than the last item in the buffer, don't add it.
  if (joint_buffer_->GetNumberOfSlices() == 0 || slice.timestamp >= joint_buffer_->GetLatestTimestamp()) {
    joint_buffer_->AddSlice(slice);
  } else {
    LOG_WARNING("Processed image is too old, discarding it.");
  }
}

auto SliceProviderImpl::GetSlice() -> std::optional<joint_model::JointProfile> {
  auto maybe_median_slice = MedianOfRecentSlices();

  if (maybe_median_slice.has_value()) {
    return maybe_median_slice.value().profile;
  }

  return std::nullopt;
}

auto SliceProviderImpl::GetTrackingSlice() -> std::optional<std::tuple<joint_tracking::JointSlice, uint64_t, double>> {
  std::tuple<joint_tracking::JointSlice, uint64_t, double> result = {};
  auto maybe_slice                                                = MedianOfRecentSlices();
  if (maybe_slice.has_value()) {
    auto slice = maybe_slice.value();
    auto area  = slice.profile.area;

    if (slice.profile.points.size() != 7) {
      return std::nullopt;
    }

    std::array<joint_tracking::Coord, 7> points = {};

    uint32_t index = 0;
    for (const auto& point : slice.profile.points) {
      points[index++] = {point.x, point.y};
    }

    joint_tracking::SliceConfidence confidence = GetConfidence(slice);
    auto tracking_slice                        = joint_tracking::JointSlice(points, confidence);
    latest_slice_ = joint_tracking::JointSlice(points, joint_tracking::SliceConfidence::NO);
    last_area_    = area;
    result        = std::make_tuple(tracking_slice, slice.timestamp.time_since_epoch().count(), area);
  } else {
    auto time_stamp = high_resolution_clock::now().time_since_epoch().count();
    result          = std::make_tuple(latest_slice_, time_stamp, last_area_);
  }

  auto confidence = get<0>(result).GetConfidence();
  switch (confidence) {
    case SliceConfidence::NO:
    case SliceConfidence::LOW:
      if (last_sent_ts_.has_value() && ((steady_clock_now_func_() - last_sent_ts_.value()) > NO_SLICE_DATA_TMO_MS)) {
        slice_degraded_ = true;
      }

      break;
    case SliceConfidence::MEDIUM:
    case SliceConfidence::HIGH:
      last_sent_ts_ = steady_clock_now_func_();
      break;
  }

  return result;
}

void SliceProviderImpl::Reset() {
  joint_buffer_->Reset();
  latest_slice_   = {};
  slice_degraded_ = false;
  last_sent_ts_   = {};
}

auto SliceProviderImpl::MedianOfRecentSlices() -> std::optional<joint_buffer::JointSlice> {
  // WARNING: only the points, and the confidence is copied
  auto recent = joint_buffer_->GetRecentSlices(RECENT_SLICES_MS);

  if (recent.size() < MIN_SLICES_FOR_MEDIAN) {
    return std::nullopt;
  }
  std::vector<double> x;
  std::transform(recent.begin(), recent.end(), std::back_inserter(x),
                 [](joint_buffer::JointSlice* slice) { return slice->profile.points[0].x; });

  const auto middle    = recent.size() / 2;
  double median_abw0_x = NAN;
  if (recent.size() & 1) {
    std::nth_element(x.begin(), x.begin() + middle, x.end());
    median_abw0_x = x[middle];
  } else {
    std::nth_element(x.begin(), x.begin() + middle, x.end());
    auto x1 = x[middle];
    std::nth_element(x.begin(), x.begin() + middle + 1, x.end());
    auto x2       = x[middle + 1];
    median_abw0_x = 0.5 * (x1 + x2);
  }
  joint_buffer::JointSlice slice;
  slice.approximation_used = false;

  int included = 0;
  std::vector<Timestamp> times;
  for (auto& old : recent) {
    if (fabs(old->profile.points[0].x - median_abw0_x) < 0.001) {
      included++;
      for (int i = 0; i < 7; i++) {
        slice.profile.points[i].x += old->profile.points[i].x;
        slice.profile.points[i].y += old->profile.points[i].y;
      }
      slice.profile.area    += old->profile.area;
      slice.num_walls_found += old->num_walls_found;
      times.push_back(old->timestamp);
      if (old->approximation_used) {
        slice.approximation_used = true;
      }
    }
  }
  if (included == 0) {
    return std::nullopt;
  }
  for (int i = 0; i < 7; i++) {
    slice.profile.points[i].x /= included;
    slice.profile.points[i].y /= included;
  }
  slice.profile.area    /= included;
  slice.num_walls_found /= included;
  std::nth_element(times.begin(), times.begin() + included / 2, times.end());
  slice.timestamp = times[included / 2];

  return slice;
}

auto SliceProviderImpl::GetConfidence(joint_buffer::JointSlice slice) -> joint_tracking::SliceConfidence {
  auto left_depth                            = slice.profile.points[0].y - slice.profile.points[1].y;
  auto right_depth                           = slice.profile.points[6].y - slice.profile.points[5].y;
  joint_tracking::SliceConfidence confidence = joint_tracking::SliceConfidence::LOW;

  if (slice.approximation_used) {
    confidence = joint_tracking::SliceConfidence::MEDIUM;
  } else if (slice.num_walls_found == 2) {
    confidence = joint_tracking::SliceConfidence::MEDIUM;

    if (left_depth > HIGH_CONFIDENCE_WALL_HEIGHT && right_depth > HIGH_CONFIDENCE_WALL_HEIGHT) {
      confidence = joint_tracking::SliceConfidence::HIGH;
    }
  } else if (slice.num_walls_found == 1) {
    confidence = joint_tracking::SliceConfidence::LOW;

    if (left_depth > MEDIUM_CONFIDENCE_WALL_HEIGHT || right_depth > MEDIUM_CONFIDENCE_WALL_HEIGHT) {
      confidence = joint_tracking::SliceConfidence::MEDIUM;
    }
  }

  return confidence;
}
}  // namespace scanner::slice_provider
