#include "settings.h"

#include <fmt/core.h>
#include <SQLiteCpp/Statement.h>
#include <SQLiteCpp/VariadicBind.h>

#include <exception>
#include <functional>
#include <nlohmann/json_fwd.hpp>
#include <optional>
#include <string>
#include <unordered_map>

#include "common/logging/application_log.h"
#include "sql_table_names.h"

namespace weld_control {

auto Settings::UseEdgeSensor() const -> bool { return use_edge_sensor_; }

auto Settings::EdgeSensorPlacementValue() const -> EdgeSensorPlacement { return edge_sensor_placement_; }

auto Settings::ToString() const -> std::string {
  return fmt::format("Settings(use_edge_sensor={}, edge_sensor_placement={})", use_edge_sensor_,
                     EdgeSensorPlacementToString(edge_sensor_placement_));
}

auto Settings::ToJson() const -> nlohmann::json {
  return {
      {"useEdgeSensor",       use_edge_sensor_                                   },
      {"edgeSensorPlacement", EdgeSensorPlacementToString(edge_sensor_placement_)},
  };
}

auto Settings::FromJson(const nlohmann::json& json) -> std::optional<Settings> {
  Settings settings;
  try {
    json.at("useEdgeSensor").get_to(settings.use_edge_sensor_);

    if (json.contains("edgeSensorPlacement")) {
      std::string placement_str;
      json.at("edgeSensorPlacement").get_to(placement_str);
      auto placement = EdgeSensorPlacementFromString(placement_str);
      if (!placement.has_value()) {
        LOG_ERROR("Invalid edge sensor placement value: {}", placement_str);
        return std::nullopt;
      }
      settings.edge_sensor_placement_ = placement.value();
    }
  } catch (const nlohmann::json::exception& e) {
    LOG_ERROR("Failed to parse Settings from JSON - exception: {}", e.what());
    return std::nullopt;
  }

  return settings;
}

void Settings::CreateTable(SQLite::Database* db) {
  if (db->tableExists(SETTINGS_TABLE_NAME)) {
    return;
  }

  std::string const cmd = fmt::format(
      "CREATE TABLE {} ("
      "id INTEGER PRIMARY KEY, "
      "use_edge_sensor INTEGER, "
      "edge_sensor_placement TEXT)",
      SETTINGS_TABLE_NAME);

  db->exec(cmd);
}

auto Settings::StoreFn() -> std::function<bool(SQLite::Database*, const Settings&)> {
  return [](SQLite::Database* db, const Settings& settings) -> bool {
    LOG_TRACE("Store Settings {}", settings.ToString());

    try {
      std::string const cmd = fmt::format("INSERT OR REPLACE INTO {} VALUES (1, ?, ?)", SETTINGS_TABLE_NAME);

      SQLite::Statement query(*db, cmd);
      SQLite::bind(query, settings.use_edge_sensor_ ? 1 : 0,
                   EdgeSensorPlacementToString(settings.edge_sensor_placement_));

      return query.exec() == 1;
    } catch (const std::exception& e) {
      LOG_ERROR("Failed to store Settings - exception: {}", e.what());
      return false;
    }
  };
}

auto Settings::GetFn() -> std::function<std::optional<Settings>(SQLite::Database*)> {
  return [](SQLite::Database* db) -> std::optional<Settings> {
    std::string cmd = fmt::format("SELECT * FROM {}", SETTINGS_TABLE_NAME);
    SQLite::Statement query(*db, cmd);

    if (!query.executeStep()) {
      return std::nullopt;
    }

    Settings settings;
    settings.use_edge_sensor_ = query.getColumn(1).getInt() == 1;

    auto placement = EdgeSensorPlacementFromString(query.getColumn(2).getString());
    if (placement.has_value()) {
      settings.edge_sensor_placement_ = placement.value();
    }

    return settings;
  };
}

auto Settings::EdgeSensorPlacementToString(EdgeSensorPlacement placement) -> std::string {
  switch (placement) {
    case EdgeSensorPlacement::LEFT:
      return "left";
    case EdgeSensorPlacement::RIGHT:
      return "right";
    default:
      return "invalid";
  }
}

auto Settings::EdgeSensorPlacementFromString(const std::string& str) -> std::optional<EdgeSensorPlacement> {
  static const std::unordered_map<std::string, EdgeSensorPlacement> MAP = {
      {"left",  EdgeSensorPlacement::LEFT },
      {"right", EdgeSensorPlacement::RIGHT},
  };

  auto it = MAP.find(str);
  return it != MAP.end() ? std::optional(it->second) : std::nullopt;
}

}  // namespace weld_control
