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

      if (distance < 0.0) {
        return {};
      }

      auto const target_position = position - distance;

      for (auto const& entry : data_) {
        if (entry.position <= target_position) {
          return entry.data;
        }
      }

      return std::nullopt;
    }

 private:
  boost::circular_buffer<Entry> data_;
};
}  // namespace common::containers
