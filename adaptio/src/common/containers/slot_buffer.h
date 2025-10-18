#pragma once
#include <cmath>
#include <memory>
#include <optional>

namespace common::containers {

/**
 * A circular buffer that maps continuous indices to discrete slots.
 *
 * SlotBuffer divides a continuous range [0, wrap_value) into a fixed number of slots.
 * Values are stored based on their index position, with indices wrapping around
 * when they exceed wrap_value.
 *
 * Example: For 100 slots over range [0, 2π):
 * - Each slot covers 2π/100 ≈ 0.0628 radians
 * - Index 0.0 maps to slot 0
 * - Index π maps to slot 49 (truncated)
 * - Index 2π maps to slot 0 (wraps around)
 *
 * The buffer only stores one value per slot. Storing to the same slot multiple
 * times will update the stored value.
 */
template <typename V>
class SlotBuffer {
 public:
  explicit SlotBuffer(size_t number_of_slots, double wrap_value)
      : data_(std::make_unique<std::optional<std::pair<double, V>>[]>(number_of_slots)),
        number_of_slots_(number_of_slots),
        wrap_value_(wrap_value),
        slot_size_(wrap_value_ / number_of_slots) {}
  ~SlotBuffer() = default;

  SlotBuffer(const SlotBuffer &other) = delete;
  SlotBuffer(SlotBuffer &other)       = delete;

  auto operator=(const SlotBuffer<V> &other) -> SlotBuffer<V> & {
    if (this != other) {
      data_            = other.data_;
      number_of_slots_ = other.number_of_slots_;
      wrap_value_      = other.wrap_value_;
      slot_size_       = other.slot_size_;
    }
  }

  auto operator=(SlotBuffer &&) -> SlotBuffer & = delete;

  auto Store(double index, V value) -> bool {
    auto const slot = CalculateSlot(index);
    if (last_slot_.has_value() && last_slot_.value() == slot) {
      return false;  // Same slot, no update
    }

    filled_slots_ += !data_[slot].has_value() ? 1 : 0;
    data_[slot]    = {index, value};
    last_slot_     = slot;

    return true;  // New slot, updated
  }

  auto Get(double index) const -> std::optional<std::pair<double, V>> { return data_[CalculateSlot(index)]; }

  auto Clear() {
    data_ = std::make_unique<std::optional<std::pair<double, V>>[]>(number_of_slots_);
    last_slot_.reset();
  }

  auto Empty() const -> bool {
    for (int i = 0; i < number_of_slots_; i++) {
      if (data_[i].has_value()) {
        return false;
      }
    }

    return true;
  }

  auto Filled() const -> bool { return filled_slots_ == number_of_slots_; }
  auto FilledSlots() const -> size_t { return filled_slots_; }
  auto Slots() const -> size_t { return number_of_slots_; }

  auto CalculateSlot(double index) const -> size_t {
    return static_cast<size_t>(std::fmod(index, wrap_value_) / slot_size_);
  }

  // NOLINTBEGIN(readability-identifier-naming)
  using iterator = std::optional<std::pair<double, V>> *;
  auto begin() -> iterator { return data_.get(); };
  auto end() -> iterator { return data_.get() + number_of_slots_; };
  // NOLINTEND(readability-identifier-naming)

 protected:
  std::unique_ptr<std::optional<std::pair<double, V>>[]> data_;
  size_t number_of_slots_{};
  double wrap_value_{};
  double slot_size_{};
  size_t filled_slots_{};
  std::optional<size_t> last_slot_{};
};

}  // namespace common::containers
