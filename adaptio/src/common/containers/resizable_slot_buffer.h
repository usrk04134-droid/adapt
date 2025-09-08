#pragma once
#include <memory>
#include <optional>
#include <utility>

#include "slot_buffer.h"

namespace common::containers {

template <typename V>
class ResizableSlotBuffer : public SlotBuffer<V> {
 public:
  explicit ResizableSlotBuffer(size_t number_of_slots, double wrap_value)
      : SlotBuffer<V>(number_of_slots, wrap_value) {}

  ~ResizableSlotBuffer() = default;

  ResizableSlotBuffer(const ResizableSlotBuffer &other) = delete;
  ResizableSlotBuffer(ResizableSlotBuffer &other)       = delete;

  auto operator=(const ResizableSlotBuffer &other) -> ResizableSlotBuffer<V> & {
    if (this != other) {
      this->data_            = other.data_;
      this->number_of_slots_ = other.number_of_slots_;
      this->wrap_value_      = other.wrap_value_;
      this->slot_size_       = other.slot_size_;
    }
  }

  auto operator=(ResizableSlotBuffer &&) -> ResizableSlotBuffer & = delete;

  auto Resize(size_t number_of_slots) {
    auto old_data = std::move(this->data_);
    this->data_   = std::make_unique<std::optional<std::pair<double, V>>[]>(number_of_slots);

    auto old_number_of_slots = this->number_of_slots_;
    this->slot_size_         = this->wrap_value_ / number_of_slots;
    this->number_of_slots_   = number_of_slots;

    for (int i = 0; i < old_number_of_slots; i++) {
      if (old_data[i].has_value()) {
        this->Store(old_data[i].value().first, old_data[i].value().second);
      }
    }
  }
};

}  // namespace common::containers
