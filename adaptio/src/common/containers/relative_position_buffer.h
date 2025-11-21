#pragma once

#include <boost/circular_buffer.hpp>

#include <cmath>
#include <limits>
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

  auto Get(double position, double distance) const -> std::optional<T> {
    if (data_.empty()) {
      return std::nullopt;
    }

    const double target_position = position - distance;
    const double epsilon         = std::numeric_limits<double>::epsilon();

    const Entry* closest_entry = nullptr;
    double smallest_diff       = std::numeric_limits<double>::infinity();

    for (const auto& entry : data_) {
      const double diff = std::abs(entry.position - target_position);

      if (closest_entry == nullptr || diff < smallest_diff ||
          (std::abs(diff - smallest_diff) <= epsilon && entry.position > closest_entry->position)) {
        closest_entry = &entry;
        smallest_diff = diff;
      }
    }

    return closest_entry ? std::optional<T>{closest_entry->data} : std::nullopt;
  }

 private:
  boost::circular_buffer<Entry> data_;
};
}  // namespace common::containers
