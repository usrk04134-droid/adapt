#include "stored_laser_torch_configuration.h"

#include <fmt/core.h>
#include <SQLiteCpp/Statement.h>
#include <SQLiteCpp/VariadicBind.h>

#include <exception>
#include <functional>
#include <nlohmann/json_fwd.hpp>
#include <optional>
#include <string>

#include "common/logging/application_log.h"

namespace calibration {

const std::string LASER_TORCH_CONFIGURATION_TABLE_NAME = "laser_torch_configuration";

auto StoredLaserTorchConfiguration::DistanceLaserTorch() const -> double { return distance_laser_torch_; }
auto StoredLaserTorchConfiguration::Stickout() const -> double { return stickout_; }
auto StoredLaserTorchConfiguration::ScannerMountAngle() const -> double { return scanner_mount_angle_; }

void StoredLaserTorchConfiguration::SetDistanceLaserTorch(double value) { distance_laser_torch_ = value; }
void StoredLaserTorchConfiguration::SetStickout(double value) { stickout_ = value; }
void StoredLaserTorchConfiguration::SetScannerMountAngle(double value) { scanner_mount_angle_ = value; }

auto StoredLaserTorchConfiguration::ToString() const -> std::string {
  return fmt::format("distance_laser_torch: {}, stickout: {}, scanner_mount_angle: {}", distance_laser_torch_,
                     stickout_, scanner_mount_angle_);
}

auto StoredLaserTorchConfiguration::ToJson() const -> nlohmann::json {
  return {
      {"distanceLaserTorch", distance_laser_torch_},
      {"stickout",           stickout_            },
      {"scannerMountAngle",  scanner_mount_angle_ }
  };
}

auto StoredLaserTorchConfiguration::FromJson(const nlohmann::json& json_obj)
    -> std::optional<StoredLaserTorchConfiguration> {
  StoredLaserTorchConfiguration config;

  try {
    json_obj.at("distanceLaserTorch").get_to(config.distance_laser_torch_);
    json_obj.at("stickout").get_to(config.stickout_);
    json_obj.at("scannerMountAngle").get_to(config.scanner_mount_angle_);
  } catch (const nlohmann::json::exception& e) {
    LOG_ERROR("Failed to parse StoredLaserTorchConfiguration from JSON - exception: {}", e.what());
    return std::nullopt;
  }

  return config;
}

auto StoredLaserTorchConfiguration::IsValid() const -> bool {
  auto ok = true;

  ok &= distance_laser_torch_ > 0.0;
  ok &= stickout_ > 0.0;
  ok &= scanner_mount_angle_ >= 0.0;

  return ok;
}

void StoredLaserTorchConfiguration::CreateTable(SQLite::Database* db) {
  if (db->tableExists(LASER_TORCH_CONFIGURATION_TABLE_NAME)) {
    return;
  }

  std::string cmd = fmt::format(
      "CREATE TABLE {} ("
      "id INTEGER PRIMARY KEY, "
      "distance_laser_torch REAL, "
      "stickout REAL, "
      "scanner_mount_angle REAL)",
      LASER_TORCH_CONFIGURATION_TABLE_NAME);

  db->exec(cmd);
}

auto StoredLaserTorchConfiguration::StoreFn()
    -> std::function<bool(SQLite::Database*, const StoredLaserTorchConfiguration&)> {
  return [](SQLite::Database* db, const StoredLaserTorchConfiguration& config) -> bool {
    LOG_TRACE("Store StoredLaserTorchConfiguration {}", config.ToString());

    try {
      std::string cmd =
          fmt::format("INSERT OR REPLACE INTO {} VALUES (1, ?, ?, ?)", LASER_TORCH_CONFIGURATION_TABLE_NAME);

      SQLite::Statement query(*db, cmd);
      SQLite::bind(query, config.distance_laser_torch_, config.stickout_, config.scanner_mount_angle_);

      return query.exec() == 1;
    } catch (const std::exception& e) {
      LOG_ERROR("Failed to store StoredLaserTorchConfiguration - exception: {}", e.what());
      return false;
    }
  };
}

auto StoredLaserTorchConfiguration::GetFn()
    -> std::function<std::optional<StoredLaserTorchConfiguration>(SQLite::Database*)> {
  return [](SQLite::Database* db) -> std::optional<StoredLaserTorchConfiguration> {
    std::string cmd = fmt::format("SELECT * FROM {}", LASER_TORCH_CONFIGURATION_TABLE_NAME);
    SQLite::Statement query(*db, cmd);

    if (!query.executeStep()) {
      return std::nullopt;
    }

    StoredLaserTorchConfiguration config;
    config.distance_laser_torch_ = query.getColumn(1).getDouble();
    config.stickout_             = query.getColumn(2).getDouble();
    config.scanner_mount_angle_  = query.getColumn(3).getDouble();

    return config;
  };
}

}  // namespace calibration
