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
  explicit RelativePositionBuffer(size_t capacity, std::optional<double> wrap_value)
      : data_(capacity), wrap_value_(wrap_value) {}

  ~RelativePositionBuffer() = default;

  auto Size() const -> size_t { return data_.size(); };
  auto Empty() const -> bool { return data_.size() == 0; };
  void Clear() { data_.clear(); };

  void Store(double position, const T& value) {
    if (data_.front().position != position) {
      data_.push_front(Entry{.position = position, .data = value});
    }
  }

  auto Get(double position) -> std::optional<T> {
    if (data_.empty()) {
      return {};
    }

    auto sum           = 0.0;
    auto last_position = data_[0].position;

    for (int i = 0; i < data_.size(); i++) {
      auto cur_position = data_[i].position;

      if (wrap_value_.has_value()) {
        sum += last_position < cur_position ? last_position + wrap_value_.value() - cur_position
                                            : last_position - cur_position;
      } else {
        sum += last_position - cur_position;
      }

      auto dist = position - sum;

      if (dist < 0.0) {
        return i > 0 ? std::optional<T>{data_[i - 1].data} : std::nullopt;
      }

      last_position = cur_position;
    }

    return data_.back().data;
  }

 private:
  boost::circular_buffer<Entry> data_;
  std::optional<double> wrap_value_;
};
}  // namespace common::containers
