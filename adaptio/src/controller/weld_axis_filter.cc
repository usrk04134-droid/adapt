#include "weld_axis_filter.h"

#include <algorithm>
#include <cstdint>

using controller::WeldAxisFilterMedianImpl;

WeldAxisFilterMedianImpl::WeldAxisFilterMedianImpl(uint8_t window_size)
    : window_size_(window_size), continuous_signal_(window_size, 0), arrange_median_(window_size) {
  if (window_size_ == 0) {
    window_size_ = 1;
  }
  if ((window_size_ % 2) == 0) {
    window_size_ += 1;
  }
}

auto WeldAxisFilterMedianImpl::ProcessSignal(float raw_value) -> float {
  for (uint8_t i = 0; i < window_size_ - 1; ++i) {
    continuous_signal_[i] = continuous_signal_[i + 1];
  }
  continuous_signal_[window_size_ - 1] = raw_value;
  FindMedian();

  return arrange_median_[window_size_ / 2];
}

void WeldAxisFilterMedianImpl::FindMedian() {
  auto mid        = window_size_ / 2;
  arrange_median_ = continuous_signal_;

  std::nth_element(arrange_median_.begin(), arrange_median_.begin() + mid, arrange_median_.end());
}

void WeldAxisFilterMedianImpl::ClearSignalBuffer() { continuous_signal_.assign(continuous_signal_.size(), 0); }

auto WeldAxisFilterMedianImpl::GetWindowSize() -> uint8_t { return window_size_; }
