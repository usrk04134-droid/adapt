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
    if (data_.front().position != position) {
      data_.push_front(Entry{.position = position, .data = value});
    }
  }

  auto Get(double position, double distance) -> std::optional<T> {
    if (data_.empty()) {
      return {};
    }

    if (distance < 0.0) {
      return {};
    }

    auto const reference_position = data_.front().position;
    auto const remaining          = distance - (position - reference_position);

    if (remaining < 0.0) {
      return {};
    }

    auto sum           = 0.0;
    auto last_position = reference_position;

    for (int i = 0; i < data_.size(); i++) {
      auto cur_position = data_[i].position;

      sum += last_position - cur_position;

      if (sum > remaining) {
        return i > 0 ? std::optional<T>{data_[i - 1].data} : std::nullopt;
      }

      last_position = cur_position;
    }

    return data_.back().data;
  }

 private:
  boost::circular_buffer<Entry> data_;
};
}  // namespace common::containers
