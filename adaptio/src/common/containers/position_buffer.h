#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

#include "common/math/math.h"

namespace common::containers {

enum class WrapMode { WRAP, NO_WRAP };

template <typename T>
class PositionBuffer {
 public:
  explicit PositionBuffer(double wrap_value, WrapMode wrap_mode = WrapMode::WRAP)
      : wrap_value_(wrap_value), wrap_mode_(wrap_mode) {}
  ~PositionBuffer() = default;

  PositionBuffer(const PositionBuffer &obj)
      : data_(obj.data_), wrap_value_(obj.wrap_value_), offset_(obj.offset_), wrap_mode_(obj.wrap_mode_) {}
  PositionBuffer(PositionBuffer &obj)
      : data_(obj.data_), wrap_value_(obj.wrap_value_), offset_(obj.offset_), wrap_mode_(obj.wrap_mode_) {}

  auto operator=(const PositionBuffer<T> &other) -> PositionBuffer<T> & {
    if (this != &other)  // not a self-assignment
    {
      data_       = other.data_;
      wrap_value_ = other.wrap_value_;
      offset_     = other.offset_;
      wrap_mode_  = other.wrap_mode_;
    }

    return *this;
  }

  auto operator=(PositionBuffer &&) -> PositionBuffer & = delete;

  /* Store data in sequential position order. The stored positions should not
   * overlap and be in the [0-max_pos) range. */
  auto Store(double pos, const T &sample) {
    if (pos >= 0.0 && pos < wrap_value_) {
      if (data_.empty()) {
        offset_ = pos;
      }
      data_.push_back({CalcPosRelativeOffset(pos), sample});
    }
  }

  /* Get stored data that is closes to pos */
  auto Get(double pos) -> std::optional<T> {
    if (pos < 0.0 || pos >= wrap_value_ || data_.empty()) {
      return std::nullopt;
    }

    if (data_.size() == 1) {
      return data_[0].second;
    }

    auto const pos_rel_offset = CalcPosRelativeOffset(pos);

    if (wrap_mode_ == WrapMode::NO_WRAP) {
      // In NO_WRAP mode, only search backwards (entries <= pos in absolute space)
      // If pos < offset_, then all stored positions are > pos, so return nullopt
      if (pos < offset_) {
        return std::nullopt;
      }
      
      // Since pos >= offset_, pos_rel_offset = pos - offset_ (no wrapping in CalcPosRelativeOffset)
      // Data is stored sequentially and relative offsets are monotonic,
      // so we can search in relative offset space directly
      auto it = std::lower_bound(data_.begin(), data_.end(), std::pair<double, T>{pos_rel_offset, {}},
                                 [](std::pair<double, T> v1, std::pair<double, T> v2) { return v1.first < v2.first; });

      // Find the last entry that is <= pos_rel_offset
      if (it != data_.end() && it->first == pos_rel_offset) {
        // Exact match
        return it->second;
      }
      
      if (it != data_.begin()) {
        // Return the entry just before it (the last entry <= pos_rel_offset)
        return (it - 1)->second;
      }
      
      // No entry found backwards (all entries are > pos_rel_offset)
      return std::nullopt;
    }

    // WRAP mode: original behavior
    auto e1 = data_.front();
    auto e2 = data_.back();
    if (pos_rel_offset > e1.first && pos_rel_offset < e2.first) {
      auto it = std::lower_bound(data_.begin(), data_.end(), std::pair<double, T>{pos_rel_offset, {}},
                                 [](std::pair<double, T> v1, std::pair<double, T> v2) { return v1.first < v2.first; });

      if (it == data_.end()) {
        return {};
      }
      e1 = *it;
      e2 = *(it - 1);
    }

    auto const dist_e1 = std::fabs(common::math::WrappedDist(pos_rel_offset, e1.first, wrap_value_));
    auto const dist_e2 = std::fabs(common::math::WrappedDist(pos_rel_offset, e2.first, wrap_value_));

    return dist_e1 < dist_e2 ? e1.second : e2.second;
  }

  auto Clear() { data_.clear(); }

  auto Size() -> size_t { return data_.size(); }

  auto Empty() -> bool { return data_.size() == 0; }

 private:
  auto CalcPosRelativeOffset(double pos) -> double {
    return pos >= offset_ ? pos - offset_ : pos - offset_ + wrap_value_;
  };
  std::vector<std::pair<double, T>> data_;
  double wrap_value_{};
  double offset_{};
  WrapMode wrap_mode_{WrapMode::WRAP};
};

}  // namespace common::containers
