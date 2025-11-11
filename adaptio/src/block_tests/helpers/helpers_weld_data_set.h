#pragma once

#include "helpers_web_hmi.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)

#include <doctest/doctest.h>
#include <SQLiteCpp/Database.h>
#include <SQLiteCpp/Statement.h>
#include <SQLiteCpp/VariadicBind.h>

#include <optional>
#include <string>

#include "helpers.h"
#include "web_hmi/web_hmi_json_helpers.h"

static const std::string WPRMSET_ADD        = "AddWeldDataSet";
static const std::string WPRMSET_ADD_RSP    = "AddWeldDataSetRsp";
static const std::string WPRMSET_REMOVE     = "RemoveWeldDataSet";
static const std::string WPRMSET_REMOVE_RSP = "RemoveWeldDataSetRsp";
static const std::string WPRMSET_GET        = "GetWeldDataSets";
static const std::string WPRMSET_GET_RSP    = "GetWeldDataSetsRsp";

inline auto GetWeldDataSets(TestFixture& fixture) {
  auto req = web_hmi::CreateMessage(WPRMSET_GET, std::nullopt, {});
  fixture.WebHmiIn()->DispatchMessage(std::move(req));

  auto const response_payload = ReceiveJsonByName(fixture, WPRMSET_GET_RSP);
  CHECK(response_payload != nullptr);

  return response_payload.at("payload");
}

[[nodiscard]] inline auto AddWeldDataSet(TestFixture& fixture, std::string const& name, int wpp_id1, int wpp_id2,
                                         bool expect_ok) {
  nlohmann::json const payload({
      {"name",     name   },
      {"ws1WppId", wpp_id1},
      {"ws2WppId", wpp_id2}
  });

  auto msg = web_hmi::CreateMessage(WPRMSET_ADD, std::nullopt, payload);
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));

  auto const response_payload = ReceiveJsonByName(fixture, WPRMSET_ADD_RSP);
  CHECK(response_payload != nullptr);

  nlohmann::json const expected({
      {"result", expect_ok ? "ok" : "fail"}
  });

  return response_payload.at("result") == expected.at("result");
}

[[nodiscard]] inline auto RemoveWeldDataSet(TestFixture& fixture, int wparams_id, bool expect_ok) {
  nlohmann::json const payload({
      {"id", wparams_id}
  });

  auto msg = web_hmi::CreateMessage(WPRMSET_REMOVE, std::nullopt, payload);
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));

  auto const response_payload = ReceiveJsonByName(fixture, WPRMSET_REMOVE_RSP);
  CHECK(response_payload != nullptr);

  nlohmann::json const expected({
      {"result", expect_ok ? "ok" : "fail"}
  });

  return response_payload.at("result") == expected.at("result");
}

inline auto EnsureWeldDataSetTable(SQLite::Database* database) -> void {
  if (database->tableExists("weld_data_sets")) {
    return;
  }

  static constexpr auto create_table_query = R"(
        CREATE TABLE weld_data_sets (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT,
            ws1_wpp_id INTEGER,
            ws2_wpp_id INTEGER
        )
    )";

  database->exec(create_table_query);
}

inline auto AddWeldDataSetToDatabase(SQLite::Database* database, std::string const& name,
                                     int weld_process_parameters_id1, int weld_process_parameters_id2) -> void {
  EnsureWeldDataSetTable(database);

  static constexpr auto insert_query = R"(
        INSERT INTO weld_data_sets (name, ws1_wpp_id, ws2_wpp_id)
        VALUES (?, ?, ?)
    )";

  SQLite::Statement query(*database, insert_query);
  SQLite::bind(query, name, weld_process_parameters_id1, weld_process_parameters_id2);
  query.exec();
}
// NOLINTEND(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)
