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

    const Entry* candidate_entry = nullptr;  // entry at or before target
    double candidate_diff        = std::numeric_limits<double>::infinity();

    const Entry* fallback_entry = nullptr;  // entry after target (no history before target)
    double fallback_diff        = std::numeric_limits<double>::infinity();

    for (const auto& entry : data_) {
      const double delta = target_position - entry.position;

      if (delta >= -epsilon) {
        const double normalized_delta = delta < 0.0 ? 0.0 : delta;
        if (candidate_entry == nullptr || normalized_delta < candidate_diff ||
            (std::abs(normalized_delta - candidate_diff) <= epsilon && entry.position > candidate_entry->position)) {
          candidate_entry = &entry;
          candidate_diff  = normalized_delta;
        }
      } else {
        const double abs_delta = std::abs(delta);
        if (fallback_entry == nullptr || abs_delta < fallback_diff ||
            (std::abs(abs_delta - fallback_diff) <= epsilon && entry.position > fallback_entry->position)) {
          fallback_entry = &entry;
          fallback_diff  = abs_delta;
        }
      }
    }

    const Entry* selected_entry = candidate_entry != nullptr ? candidate_entry : fallback_entry;
    return selected_entry ? std::optional<T>{selected_entry->data} : std::nullopt;
  }

 private:
  boost::circular_buffer<Entry> data_;
};
}  // namespace common::containers
