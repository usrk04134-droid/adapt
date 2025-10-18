#pragma once

#include <SQLiteCpp/Database.h>

#include <functional>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "calibration_solver.h"
#include "common/types/vector_3d.h"

namespace calibration {

class StoredCalibrationResult {
 public:
  auto WeldObjectRotationAxis() const -> common::Vector3D;
  auto RotationCenter() const -> common::Vector3D;
  auto TorchToLpcsTranslation() const -> common::Vector3D;
  auto ResidualStandardError() const -> double;
  auto WeldObjectRadius() const -> double;

  void SetWeldObjectRotationAxis(const common::Vector3D& value);
  void SetRotationCenter(const common::Vector3D& value);
  void SetTorchToLpcsTranslation(const common::Vector3D& value);
  void SetResidualStandardError(double value);
  void SetWeldObjectRadius(double value);

  auto ToString() const -> std::string;
  auto ToJson() const -> nlohmann::json;
  static auto FromJson(const nlohmann::json& json_obj) -> std::optional<StoredCalibrationResult>;
  static auto FromCalibrationResult(const CalibrationResult& data, double weld_object_radius)
      -> StoredCalibrationResult;

  static void CreateTable(SQLite::Database* db);
  static auto StoreFn() -> std::function<bool(SQLite::Database*, const StoredCalibrationResult&)>;
  static auto GetFn() -> std::function<std::optional<StoredCalibrationResult>(SQLite::Database*)>;

  // To support clear when LaserTorchConfiguration is updated
  static auto ClearFn() -> std::function<bool(SQLite::Database*)>;

  auto IsValid() const -> bool;

 private:
  common::Vector3D weld_object_rotation_axis_{};
  common::Vector3D rotation_center_{};
  common::Vector3D torch_to_lpcs_translation_{};
  double residual_standard_error_{};
  double weld_object_radius_{};
};

}  // namespace calibration
