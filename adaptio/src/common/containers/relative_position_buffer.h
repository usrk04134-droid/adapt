#pragma once

#include <boost/circular_buffer.hpp>
#include <cmath>
#include <optional>

namespace common::containers {

enum Round { CLOSEST, MIN };

template <typename T>
class RelativePositionBuffer {
 public:
  struct Entry {
    double position{0.0};
    T data;
  };

  class Slice {
   public:
    Slice() = default;
    Slice(const boost::circular_buffer<Entry>::const_iterator start_it, uint32_t size)
        : iterator_(start_it), size_(size) {}
    // NOLINTNEXTLINE readability-identifier-naming
    auto begin() const -> boost::circular_buffer<Entry>::const_iterator { return iterator_; }
    // NOLINTNEXTLINE readability-identifier-naming
    auto end() const -> boost::circular_buffer<Entry>::const_iterator { return iterator_ + size_; }
    auto Size() const -> uint32_t { return size_; }
    auto Empty() const -> bool { return size_ == 0; }

   private:
    boost::circular_buffer<Entry>::const_iterator iterator_;
    uint32_t size_{0};
  };

  explicit RelativePositionBuffer(size_t capacity) : data_(capacity) {}
  ~RelativePositionBuffer() = default;

  auto Size() const -> size_t { return data_.size(); };
  auto Empty() const -> bool { return data_.size() == 0; };
  void Clear() { data_.clear(); };

  // Stores data where position is between 0-2*pi and is wrapped
  void Store(double position, const T& value) { data_.push_front(Entry{.position = position, .data = value}); }

  // Get data between two positions.
  // start_position and stop_position are related to the last stored position
  // E.g. to get data from the latest turn: Get(0.0, 2*pi)
  // To get data from second latest turn: Get(2*pi, 4*pi)
  // Returns: Slice that can be used to iterate over the the data
  auto Get(double start_position, double stop_position) -> Slice {
    if (data_.empty()) {
      return RelativePositionBuffer<T>::Slice(data_.begin(), 0);
    }

    auto data        = GetIndex(start_position, stop_position);
    auto start_index = get<0>(data);
    auto stop_index  = get<1>(data);

    return RelativePositionBuffer<T>::Slice(data_.begin() + start_index, stop_index - start_index);
  }

  // Get data for a position
  // To get latest stored data: Get(0.0, Round::CLOSEST/MIN)
  // To get data from one turn ago:  Get(2*pi)
  // The position argument is relative from latest stored position
  auto Get(double position, Round round = Round::CLOSEST) -> std::optional<T> {
    if (data_.empty()) {
      return {};
    }

    auto lin_angle     = 0.0;
    auto last_position = data_[0].position;
    auto last_dist     = 0.0;

    for (int i = 0; i < data_.size(); i++) {
      auto cur_position = data_[i].position;

      lin_angle += last_position < cur_position ? last_position + 2.0 * std::numbers::pi - cur_position
                                                : last_position - cur_position;

      auto dist = position - lin_angle;

      if (round == Round::MIN) {
        if (dist < 0.0) {
          return i > 0 ? std::optional<T>{data_[i - 1].data} : std::nullopt;
        }
      } else {  // Round::CLOSEST
        if (dist < 0.0) {
          return last_dist < std::abs(dist) ? data_[i - 1].data : data_[i].data;
        }
      }

      last_position = cur_position;
      last_dist     = dist;
    }

    return data_.back().data;
  }

 private:
  auto GetIndex(double start_position, double stop_position) -> std::tuple<uint32_t, uint32_t> {
    uint32_t start_index = -1;
    uint32_t stop_index  = -1;
    auto lin_position    = 0.0;
    auto last_position   = !data_.empty() ? data_[0].position : 0.0;

    for (int i = 0; i < data_.size(); i++) {
      auto cur_position = data_[i].position;

      lin_position += last_position < cur_position ? last_position + 2.0 * std::numbers::pi - cur_position
                                                   : last_position - cur_position;

      if (start_index == -1 && lin_position >= start_position) {
        start_index = i;
      }

      if (lin_position > stop_position) {
        break;
      }
      stop_index = i + 1;

      last_position = cur_position;
    }

    // Check if start_position is not within the range
    if (start_index == -1) {
      return {0, 0};
    }

    return {start_index, stop_index};
  }

  boost::circular_buffer<Entry> data_;
};
}  // namespace common::containers
