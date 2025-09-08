#pragma once

#include <optional>

namespace storage {

template <typename T>
class SingleStorage {
 public:
  virtual ~SingleStorage() = default;

  virtual auto Store(const T& data) -> bool    = 0;
  virtual auto Get() const -> std::optional<T> = 0;
};

}  // namespace storage
