#pragma once

#include <vector>

namespace storage {

template <typename T>
class MultiStorage {
 public:
  virtual ~MultiStorage() = default;

  virtual auto Store(const T& data) -> bool     = 0;
  virtual auto Remove(int id) -> bool           = 0;
  virtual auto GetAll() const -> std::vector<T> = 0;
};

}  // namespace storage
