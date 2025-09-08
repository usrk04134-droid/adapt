#pragma once

#include <SQLiteCpp/Database.h>

#include <functional>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

namespace weld_control {

class Settings {
 public:
  enum class EdgeSensorPlacement {
    LEFT,
    RIGHT,
  };

  Settings() = default;

  auto UseEdgeSensor() const -> bool;
  auto EdgeSensorPlacementValue() const -> EdgeSensorPlacement;
  auto ToString() const -> std::string;
  auto ToJson() const -> nlohmann::json;
  static auto FromJson(const nlohmann::json&) -> std::optional<Settings>;

  static void CreateTable(SQLite::Database*);
  static auto StoreFn() -> std::function<bool(SQLite::Database*, const Settings&)>;
  static auto GetFn() -> std::function<std::optional<Settings>(SQLite::Database*)>;

 private:
  static auto EdgeSensorPlacementToString(EdgeSensorPlacement placement) -> std::string;
  static auto EdgeSensorPlacementFromString(const std::string&) -> std::optional<EdgeSensorPlacement>;

  bool use_edge_sensor_{true};
  EdgeSensorPlacement edge_sensor_placement_{EdgeSensorPlacement::RIGHT};
};

}  // namespace weld_control
