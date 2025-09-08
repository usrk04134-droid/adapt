#pragma once

#include <SQLiteCpp/Database.h>

#include <functional>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

namespace calibration {

class StoredLaserTorchConfiguration {
 public:
  auto DistanceLaserTorch() const -> double;
  auto Stickout() const -> double;
  auto ScannerMountAngle() const -> double;

  void SetDistanceLaserTorch(double value);
  void SetStickout(double value);
  void SetScannerMountAngle(double value);

  auto ToString() const -> std::string;
  auto ToJson() const -> nlohmann::json;
  static auto FromJson(const nlohmann::json& json_obj) -> std::optional<StoredLaserTorchConfiguration>;

  static void CreateTable(SQLite::Database* db);
  static auto StoreFn() -> std::function<bool(SQLite::Database*, const StoredLaserTorchConfiguration&)>;
  static auto GetFn() -> std::function<std::optional<StoredLaserTorchConfiguration>(SQLite::Database*)>;

  auto IsValid() const -> bool;

 private:
  double distance_laser_torch_{};
  double stickout_{};
  double scanner_mount_angle_{};
};

}  // namespace calibration
