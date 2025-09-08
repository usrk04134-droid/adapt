#pragma once

#include <SQLiteCpp/Database.h>

#include <functional>
#include <utility>

#include "common/logging/application_log.h"
#include "multi_storage.h"

namespace storage {

using Sql = SQLite::Database;

template <typename T>
class SqlMultiStorage : public MultiStorage<T> {
 public:
  using OnInit   = std::function<void(Sql*)>;
  using OnStore  = std::function<bool(Sql*, const T&)>;
  using OnRemove = std::function<bool(Sql*, int)>;
  using OnGetAll = std::function<std::vector<T>(Sql*)>;

  SqlMultiStorage(Sql* database, const OnInit& on_init, OnStore on_store, OnRemove on_remove, OnGetAll on_get_all)
      : database_(database),
        on_store_(std::move(on_store)),
        on_remove_(std::move(on_remove)),
        on_get_all_(std::move(on_get_all)) {
    on_init(database_);
    UpdateCache();
  };

  auto Store(const T& data) -> bool override {
    if (!IsValid(data)) {
      LOG_ERROR("Store - Invalid data: {}", data.ToString());
      return false;
    }

    auto const ok = on_store_(database_, data);
    if (!ok) {
      return false;
    }

    /* read the stored data to make sure that infomration that were not stored are not cached */
    return UpdateCache();
  }

  auto Remove(int id) -> bool override {
    auto was_removed = on_remove_(database_, id);
    if (was_removed) {
      UpdateCache();
    }
    return was_removed;
  }

  auto GetAll() const -> std::vector<T> override { return cache_; };

 private:
  auto IsValid(const T& data) -> bool {
    /* check if T has a IsValid method, if it does use it and if it doesn't set valid to true */
    constexpr bool TypeProvidesIsValid = requires(const T& tt) { tt.IsValid(); };

    if constexpr (TypeProvidesIsValid) {
      return data.IsValid();
    }

    return true;
  };

  auto UpdateCache() -> bool {
    auto const data = on_get_all_(database_);

    for (const auto& item : data) {
      if (!IsValid(item)) {
        return false;
      }
    }

    cache_ = data;

    return true;
  }

  SQLite::Database* database_;
  OnStore on_store_;
  OnRemove on_remove_;
  OnGetAll on_get_all_;

  std::vector<T> cache_;
};

}  // namespace storage
