#pragma once

#include <SQLiteCpp/Database.h>

#include <functional>
#include <utility>

#include "common/logging/application_log.h"
#include "single_storage.h"

namespace storage {

using Sql = SQLite::Database;
template <typename T>
class SqlSingleStorage : public SingleStorage<T> {
 public:
  using OnInit   = std::function<void(Sql*)>;
  using OnStore  = std::function<bool(Sql*, const T&)>;
  using OnGet    = std::function<std::optional<T>(Sql*)>;
  using OnUpdate = std::function<void()>;

  SqlSingleStorage(Sql* database, const OnInit& on_init, OnStore on_store, OnGet on_get, OnUpdate on_update = {})
      : database_(database), on_store_(std::move(on_store)), on_get_(std::move(on_get)) {
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

    return UpdateCache();
  }

  auto Clear() -> bool {
    if constexpr (requires { T::ClearFn(); }) {
      auto success = T::ClearFn()(database_);
      if (success) {
        UpdateCache();
      }
      return success;
    } else {
      static_assert(false, "Type T must implement static ClearFn()");
    }
  }

  auto Get() const -> std::optional<T> override { return cache_; };

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
    auto const data = on_get_(database_);

    if (data.has_value() && !IsValid(data.value())) {
      return false;
    }

    cache_ = data;

    return true;
  }

  SQLite::Database* database_;
  OnStore on_store_;
  OnGet on_get_;

  std::optional<T> cache_{};
};

}  // namespace storage
