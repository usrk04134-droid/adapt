#include <fmt/core.h>
#include <SQLiteCpp/Database.h>
#include <SQLiteCpp/Statement.h>
#include <SQLiteCpp/VariadicBind.h>

#include <chrono>
#include <cstdio>
#include <string>

static std::string const STR1 = "this is just a test!";

void SqlitePerformanceTest() {
  fmt::println("SQLite Performance test");

  std::string const path = "tmp.db3";
  std::remove(path.c_str());

  int const operations = 1000;

  /* Setup */
  // NOLINTNEXTLINE(hicpp-signed-bitwise)
  auto database = SQLite::Database(path, SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE);
  database.exec("CREATE TABLE benchmark_table (val1 INTEGER, val2 varchar(128))");

  // database.exec("PRAGMA synchronous=off");

  auto const time1 = std::chrono::high_resolution_clock::now();

  /* Test insert */
  for (int i = 0; i < operations; i++) {
    auto query = SQLite::Statement(database, "INSERT INTO benchmark_table VALUES (?, ?)");
    SQLite::bind(query, i, STR1);
    if (query.exec() != 1) {
      fmt::println(stdout, "query failed!");
      return;
    }
  }
  auto const time2 = std::chrono::high_resolution_clock::now();

  /* Test update */
  for (int i = 0; i < operations; i++) {
    auto query = SQLite::Statement(database, "UPDATE benchmark_table SET val2=(?) WHERE val1=(?)");
    SQLite::bind(query, STR1, i);
    if (query.exec() != 1) {
      fmt::println(stdout, "query failed!");
      return;
    }
  }
  auto const time3 = std::chrono::high_resolution_clock::now();

  /* Test get/select by integer key */
  for (int i = 0; i < operations; i++) {
    SQLite::Statement query(database, "SELECT val2 FROM benchmark_table WHERE val1=(?)");
    SQLite::bind(query, i);
    while (query.executeStep()) {
      if (query.getColumn(0).getString() != STR1) {
        fmt::println(stdout, "query failed!");
        return;
      }
    }
  }
  auto const time4 = std::chrono::high_resolution_clock::now();

  auto const elapsed_insert = std::chrono::duration_cast<std::chrono::milliseconds>(time2 - time1);
  auto const elapsed_update = std::chrono::duration_cast<std::chrono::milliseconds>(time3 - time2);
  auto const elapsed_select = std::chrono::duration_cast<std::chrono::milliseconds>(time4 - time3);

  fmt::println(stdout, "  INSERT elapsed: {} ms", elapsed_insert.count());
  fmt::println(stdout, "  UPDATE elapsed: {} ms", elapsed_update.count());
  fmt::println(stdout, "  SELECT elapsed: {} ms", elapsed_select.count());

  std::remove(path.c_str());
}
