#pragma once

#include <boost/circular_buffer.hpp>
#include <cmath>
#include <iostream>
#include <optional>
namespace common::containers {

template <typename T>
class RelativePositionBuffer {
 public:
  struct Entry {
    double position{0.0};
    T data;
  };

  explicit RelativePositionBuffer(size_t capacity) : data_(capacity) {}

  ~RelativePositionBuffer() = default;

  auto Size() const -> size_t { return data_.size(); };
  auto Empty() const -> bool { return data_.size() == 0; };
  void Clear() { data_.clear(); };

  void Store(double position, const T& value) {
    if (data_.empty() || data_.front().position != position) {
      data_.push_front(Entry{.position = position, .data = value});
    }
  }

  auto Get(double position, double distance) -> std::optional<T> {
    if (data_.empty()) {
      return {};
    }

    auto const target_position = position - distance;

    auto closest_it = data_.begin();
    auto min_diff   = std::abs(closest_it->position - target_position);

    for (auto it = data_.begin() + 1; it != data_.end(); ++it) {
      auto const diff = std::abs(it->position - target_position);
      if (diff > min_diff) {
        break;
      }
      min_diff   = diff;
      closest_it = it;
    }

    return closest_it->data;
  }

 private:
  boost::circular_buffer<Entry> data_;
};
}  // namespace common::containers
