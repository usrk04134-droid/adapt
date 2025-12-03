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
  explicit PositionBuffer(double wrap_value, WrapMode mode = WrapMode::WRAP)
      : wrap_value_(wrap_value), wrap_enabled_(mode == WrapMode::WRAP) {}
  ~PositionBuffer() = default;

  PositionBuffer(const PositionBuffer &obj)
      : data_(obj.data_), wrap_value_(obj.wrap_value_), offset_(obj.offset_), wrap_enabled_(obj.wrap_enabled_) {};
  PositionBuffer(PositionBuffer &obj)
      : data_(obj.data_), wrap_value_(obj.wrap_value_), offset_(obj.offset_), wrap_enabled_(obj.wrap_enabled_) {};

  auto operator=(const PositionBuffer<T> &other) -> PositionBuffer<T> & {
    if (this != &other)  // not a self-assignment
    {
      data_       = other.data_;
      wrap_value_ = other.wrap_value_;
      offset_     = other.offset_;
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

    if (!wrap_enabled_) {
      return e1.first < e2.first ? e2.second : e1.second;
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
  bool wrap_enabled_{true};
};

}  // namespace common::containers
