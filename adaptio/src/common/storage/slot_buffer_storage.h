#pragma once

#include <fmt/core.h>
#include <SQLiteCpp/Database.h>
#include <SQLiteCpp/Exception.h>
#include <SQLiteCpp/SQLiteCpp.h>
#include <SQLiteCpp/Statement.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <type_traits>

#include "common/containers/slot_buffer.h"
#include "common/logging/application_log.h"

namespace common::storage {

template <typename T>
class SlotBufferStorage {
  static_assert(std::is_same_v<decltype(std::declval<T>().ToJson()), nlohmann::json>,
                "T must have ToJson() method returning nlohmann::json");
  static_assert(std::is_same_v<decltype(T::FromJson(std::declval<nlohmann::json>())), std::optional<T>>,
                "T must have static FromJson(nlohmann::json) method returning std::optional<T>");

 public:
  explicit SlotBufferStorage(SQLite::Database* db, std::string table_prefix)
      : db_(db), table_prefix_(std::move(table_prefix)) {
    data_table_name_ = table_prefix_ + "_data";
    meta_table_name_ = table_prefix_ + "_metadata";
    LoadDataFromDatabase();
  }

  auto Init(size_t slots, double wrap_value) -> void {
    cache_      = std::make_unique<common::containers::SlotBuffer<T>>(slots, wrap_value);
    wrap_value_ = wrap_value;
    CreateTables(slots);
  }

  auto Store(double pos, T const& value) -> void {
    if (cache_ == nullptr || !cache_->Store(pos, value)) {
      return;  // Same slot as last time, skip database operation
    }

    if (db_ != nullptr) {
      try {
        auto json_data = value.ToJson();
        auto slot      = cache_->CalculateSlot(pos);

        auto const cmd = fmt::format(
            "INSERT OR REPLACE INTO {}"
            " (slot, position, data) "
            "VALUES (?, ?, ?)",
            data_table_name_);

        SQLite::Statement query(*db_, cmd);
        query.bind(1, static_cast<int64_t>(slot));
        query.bind(2, pos);
        query.bind(3, json_data.dump());
        query.exec();
      } catch (const SQLite::Exception& e) {
        LOG_ERROR("Database error in Store: {}", e.what());
      }
    }
  }

  auto Get(double pos) const -> std::optional<std::pair<double, T>> {
    return cache_ != nullptr ? cache_->Get(pos) : std::nullopt;
  }

  auto Clear() -> void {
    cache_ = nullptr;

    if (db_ != nullptr) {
      try {
        if (db_->tableExists(meta_table_name_)) {
          auto const cmd = fmt::format("DROP TABLE {}", meta_table_name_);
          db_->exec(cmd);
        }

        if (db_->tableExists(data_table_name_)) {
          auto const cmd = fmt::format("DROP TABLE {}", data_table_name_);
          db_->exec(cmd);
        }
      } catch (const SQLite::Exception& e) {
        LOG_ERROR("Database error in Clear: {}", e.what());
      }
    }
  }

  auto Empty() const -> bool { return cache_ != nullptr ? cache_->Empty() : true; }

  auto Filled() const -> bool { return cache_ != nullptr ? cache_->Filled() : false; }

  auto FilledSlots() const -> size_t { return cache_ != nullptr ? cache_->FilledSlots() : 0; }

  auto Slots() const -> size_t { return cache_ != nullptr ? cache_->Slots() : 0; }

  auto FillRatio() const -> double {
    return cache_ != nullptr ? static_cast<double>(cache_->FilledSlots()) / static_cast<double>(cache_->Slots()) : 0.0;
  }

  auto Available() const -> bool { return cache_ != nullptr; }

 private:
  auto CreateTables(size_t slots) -> void {
    if (db_ == nullptr) {
      return;
    }

    try {
      // Create metadata table
      if (!db_->tableExists(meta_table_name_)) {
        {
          auto const cmd = fmt::format(
              "CREATE TABLE {} ("
              "id INTEGER PRIMARY KEY, "
              "slots INTEGER NOT NULL, "
              "wrap_value REAL NOT NULL"
              ")",
              meta_table_name_);
          db_->exec(cmd);
        }

        {
          // Insert metadata
          auto const cmd = fmt::format("INSERT INTO {} (id, slots, wrap_value) VALUES (1, ?, ?)", meta_table_name_);
          SQLite::Statement query(*db_, cmd);
          query.bind(1, static_cast<int64_t>(slots));
          query.bind(2, wrap_value_);
          query.exec();
        }
      }

      // Create data table
      if (!db_->tableExists(data_table_name_)) {
        auto const cmd = fmt::format(
            "CREATE TABLE {} ("
            "slot INTEGER PRIMARY KEY, "
            "position REAL NOT NULL, "
            "data TEXT NOT NULL"
            ")",
            data_table_name_);
        db_->exec(cmd);
      }
    } catch (const SQLite::Exception& e) {
      LOG_ERROR("Database error in CreateTables: {}", e.what());
    }
  }

  auto LoadDataFromDatabase() -> void {
    if (db_ == nullptr) {
      return;
    }

    try {
      if (!db_->tableExists(meta_table_name_)) {
        return;
      }

      auto const cmd = fmt::format("SELECT slots, wrap_value FROM {}", meta_table_name_);
      SQLite::Statement query(*db_, cmd);

      if (!query.executeStep()) {
        return;
      }

      auto const slots = static_cast<size_t>(query.getColumn(0).getInt64());
      wrap_value_      = query.getColumn(1).getDouble();
      cache_           = std::make_unique<common::containers::SlotBuffer<T>>(slots, wrap_value_);
    } catch (const SQLite::Exception& e) {
      LOG_ERROR("Database error loading metadata: {}", e.what());
      return;
    }

    try {
      if (!db_->tableExists(data_table_name_)) {
        return;
      }

      auto const cmd = fmt::format("SELECT slot, position, data FROM {}", data_table_name_);
      SQLite::Statement query(*db_, cmd);

      while (query.executeStep()) {
        // Skip slot column (0) since we only need position and data for loading
        auto position  = query.getColumn(1).getDouble();
        auto json_data = query.getColumn(2).getString();

        try {
          auto const parsed_value = T::FromJson(nlohmann::json::parse(json_data));

          if (parsed_value.has_value()) {
            cache_->Store(position, parsed_value.value());
          }
        } catch (const nlohmann::json::exception& e) {
          LOG_ERROR("JSON parsing error for data: {}", e.what());
          continue;
        }
      }
    } catch (const SQLite::Exception& e) {
      LOG_ERROR("Database error loading data: {}", e.what());
    }

    if (cache_ != nullptr) {
      LOG_INFO("{} loaded: {}/{} slots filled ({:.1f}%)\n", table_prefix_, cache_->FilledSlots(), cache_->Slots(),
               FillRatio() * 100.0);
    }
  }

  std::unique_ptr<common::containers::SlotBuffer<T>> cache_;
  SQLite::Database* db_;
  std::string table_prefix_;
  std::string data_table_name_;
  std::string meta_table_name_;
  double wrap_value_{};
};

}  // namespace common::storage
