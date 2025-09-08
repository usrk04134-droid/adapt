#include "common/storage/slot_buffer_storage.h"

#include <doctest/doctest.h>
#include <fmt/core.h>
#include <SQLiteCpp/Database.h>
#include <SQLiteCpp/Statement.h>

#include <cstddef>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>
#include <numbers>
#include <optional>
#include <string>

// NOLINTBEGIN(*-magic-numbers, hicpp-signed-bitwise)

namespace {

// Simple test data structure that implements ToJson/FromJson
struct TestData {
  double value;
  std::string name;

  auto ToJson() const -> nlohmann::json {
    return nlohmann::json{
        {"value", value},
        {"name",  name }
    };
  }

  static auto FromJson(const nlohmann::json& json_obj) -> std::optional<TestData> {
    try {
      return TestData{.value = json_obj.at("value").get<double>(), .name = json_obj.at("name").get<std::string>()};
    } catch (const nlohmann::json::exception&) {
      return std::nullopt;
    }
  }
};

// Helper function to create test data
auto CreateTestData(double val, const std::string& n) -> TestData { return TestData{.value = val, .name = n}; }

// Helper to setup database with test data
auto SetupDatabaseWithTestData(SQLite::Database& db, size_t slots, double wrap_value) -> void {
  // Create metadata table
  std::string metadata_table = "test_metadata";
  db.exec(
      fmt::format("CREATE TABLE {} ("
                  "id INTEGER PRIMARY KEY, "
                  "slots INTEGER NOT NULL, "
                  "wrap_value REAL NOT NULL"
                  ")",
                  metadata_table));

  // Insert metadata
  SQLite::Statement meta_query(db,
                               fmt::format("INSERT INTO {} (id, slots, wrap_value) VALUES (1, ?, ?)", metadata_table));
  meta_query.bind(1, static_cast<int64_t>(slots));
  meta_query.bind(2, wrap_value);
  meta_query.exec();

  // Create data table
  db.exec(
      "CREATE TABLE test_data ("
      "slot INTEGER PRIMARY KEY, "
      "position REAL NOT NULL, "
      "data TEXT NOT NULL"
      ")");

  // Insert test data
  auto test_data_0 = CreateTestData(5.0, "first");
  auto test_data_1 = CreateTestData(10.0, "second");
  auto test_data_2 = CreateTestData(15.0, "third");

  SQLite::Statement data_query(db, "INSERT INTO test_data (slot, position, data) VALUES (?, ?, ?)");

  // Add some test data points (calculate slot numbers based on positions)
  auto const slot_size = wrap_value / slots;

  data_query.bind(1, static_cast<int64_t>(0.0 / slot_size));
  data_query.bind(2, 0.0);
  data_query.bind(3, test_data_0.ToJson().dump());
  data_query.exec();

  data_query.reset();
  data_query.bind(1, static_cast<int64_t>(1.0 / slot_size));
  data_query.bind(2, 1.0);
  data_query.bind(3, test_data_1.ToJson().dump());
  data_query.exec();

  data_query.reset();
  data_query.bind(1, static_cast<int64_t>(2.0 / slot_size));
  data_query.bind(2, 2.0);
  data_query.bind(3, test_data_2.ToJson().dump());
  data_query.exec();
}

}  // namespace

TEST_SUITE("SlotBufferStorage") {
  TEST_CASE("basic_operations") {
    SQLite::Database db(":memory:", SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE);

    common::storage::SlotBufferStorage<TestData> buffer(&db, "test");

    CHECK_EQ(buffer.Available(), false);
    CHECK_EQ(buffer.Empty(), true);
    CHECK_EQ(buffer.Filled(), false);
    CHECK_EQ(buffer.FilledSlots(), 0);
    CHECK_EQ(buffer.Slots(), 0);
    CHECK_EQ(buffer.FillRatio(), 0.0);

    size_t slots      = 100;
    double wrap_value = 2 * std::numbers::pi;
    buffer.Init(slots, wrap_value);

    CHECK_EQ(buffer.Available(), true);
    CHECK_EQ(buffer.Empty(), true);
    CHECK_EQ(buffer.Filled(), false);
    CHECK_GT(buffer.Slots(), 0);
    CHECK_EQ(buffer.FilledSlots(), 0);
    CHECK_EQ(buffer.FillRatio(), 0.0);

    auto test_data = CreateTestData(5.0, "test");
    buffer.Store(0.0, test_data);

    CHECK_EQ(buffer.Empty(), false);
    CHECK_EQ(buffer.FilledSlots(), 1);
    CHECK_GT(buffer.FillRatio(), 0.0);

    auto retrieved = buffer.Get(0.0);
    CHECK(retrieved.has_value());
    CHECK_EQ(retrieved->first, 0.0);
    CHECK_EQ(retrieved->second.value, 5.0);
    CHECK_EQ(retrieved->second.name, "test");

    auto non_existent = buffer.Get(10.0);
    CHECK_FALSE(non_existent.has_value());
  }

  TEST_CASE("database_persistence") {
    SQLite::Database db(":memory:", SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE);

    size_t slots      = 100;
    double wrap_value = 2 * std::numbers::pi;
    auto test_data    = CreateTestData(25.0, "persistent");

    {
      common::storage::SlotBufferStorage<TestData> buffer(&db, "test");
      buffer.Init(slots, wrap_value);
      buffer.Store(1.5, test_data);

      CHECK_EQ(buffer.FilledSlots(), 1);

      auto retrieved = buffer.Get(1.5);
      CHECK(retrieved.has_value());
      CHECK_EQ(retrieved->second.value, 25.0);
      CHECK_EQ(retrieved->second.name, "persistent");
    }

    {
      common::storage::SlotBufferStorage<TestData> buffer(&db, "test");
      CHECK_EQ(buffer.Available(), true);
      CHECK_EQ(buffer.FilledSlots(), 1);
      CHECK_GT(buffer.FillRatio(), 0.0);

      auto retrieved = buffer.Get(1.5);
      CHECK(retrieved.has_value());
      CHECK_EQ(retrieved->second.value, 25.0);
      CHECK_EQ(retrieved->second.name, "persistent");
    }
  }

  TEST_CASE("load_from_existing_database") {
    SQLite::Database db(":memory:", SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE);

    size_t slots      = 100;
    double wrap_value = 2 * std::numbers::pi;
    SetupDatabaseWithTestData(db, slots, wrap_value);

    common::storage::SlotBufferStorage<TestData> buffer(&db, "test");

    CHECK_EQ(buffer.Available(), true);
    CHECK_EQ(buffer.FilledSlots(), 3);
    CHECK_GT(buffer.FillRatio(), 0.0);

    auto data0 = buffer.Get(0.0);
    CHECK(data0.has_value());
    CHECK_EQ(data0->second.value, 5.0);
    CHECK_EQ(data0->second.name, "first");

    auto data1 = buffer.Get(1.0);
    CHECK(data1.has_value());
    CHECK_EQ(data1->second.value, 10.0);
    CHECK_EQ(data1->second.name, "second");

    auto data2 = buffer.Get(2.0);
    CHECK(data2.has_value());
    CHECK_EQ(data2->second.value, 15.0);
    CHECK_EQ(data2->second.name, "third");
  }

  TEST_CASE("clear") {
    SQLite::Database db(":memory:", SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE);

    common::storage::SlotBufferStorage<TestData> buffer(&db, "test");
    size_t slots      = 100;
    double wrap_value = 2 * std::numbers::pi;
    buffer.Init(slots, wrap_value);

    auto test_data = CreateTestData(30.0, "clear_test");
    buffer.Store(0.0, test_data);
    buffer.Store(1.0, test_data);

    CHECK_EQ(buffer.FilledSlots(), 2);
    CHECK_EQ(buffer.Empty(), false);

    buffer.Clear();

    CHECK_EQ(buffer.Available(), false);
    CHECK_EQ(buffer.Empty(), true);
    CHECK_EQ(buffer.FilledSlots(), 0);
    CHECK_EQ(buffer.FillRatio(), 0.0);

    CHECK_FALSE(db.tableExists("test_data"));
    CHECK_FALSE(db.tableExists("test_metadata"));
  }

  TEST_CASE("same_slot_replacement") {
    SQLite::Database db(":memory:", SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE);

    common::storage::SlotBufferStorage<TestData> buffer(&db, "test");
    size_t slots      = 100;
    double wrap_value = 2 * std::numbers::pi;
    buffer.Init(slots, wrap_value);

    auto test_data1 = CreateTestData(10.0, "first");
    auto test_data2 = CreateTestData(20.0, "second");

    buffer.Store(0.0, test_data1);
    CHECK_EQ(buffer.FilledSlots(), 1);

    // Store in same slot - should replace
    buffer.Store(0.0, test_data2);
    CHECK_EQ(buffer.FilledSlots(), 1);

    auto retrieved = buffer.Get(0.0);
    CHECK(retrieved.has_value());
    CHECK_EQ(retrieved->second.value, 10.0);  // First value stored wins (cache behavior)
    CHECK_EQ(retrieved->second.name, "first");
  }

  TEST_CASE("null_database") {
    common::storage::SlotBufferStorage<TestData> buffer(nullptr, "test");

    CHECK_EQ(buffer.Available(), false);
    CHECK_EQ(buffer.Empty(), true);

    buffer.Init(100, 2 * std::numbers::pi);
    CHECK_EQ(buffer.Available(), true);

    auto test_data = CreateTestData(5.0, "null_db");
    buffer.Store(0.0, test_data);

    auto retrieved = buffer.Get(0.0);
    CHECK(retrieved.has_value());
    CHECK_EQ(retrieved->second.value, 5.0);
    CHECK_EQ(retrieved->second.name, "null_db");
  }
}
// NOLINTEND(*-magic-numbers, hicpp-signed-bitwise)