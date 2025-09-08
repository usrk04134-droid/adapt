#include "stored_calibration_result.h"

#include <common/types/vector_3d_helpers.h>
#include <fmt/core.h>
#include <SQLiteCpp/Statement.h>
#include <SQLiteCpp/VariadicBind.h>

#include <cmath>
#include <exception>
#include <functional>
#include <nlohmann/json_fwd.hpp>
#include <optional>
#include <string>

#include "calibration/src/calibration_solver.h"
#include "common/logging/application_log.h"
#include "common/types/vector_3d.h"

namespace calibration {

const std::string CALIBRATION_RESULT_TABLE_NAME = "calibration_result";

auto StoredCalibrationResult::WeldObjectRotationAxis() const -> common::Vector3D { return weld_object_rotation_axis_; }

auto StoredCalibrationResult::RotationCenter() const -> common::Vector3D { return rotation_center_; }

auto StoredCalibrationResult::TorchToLpcsTranslation() const -> common::Vector3D { return torch_to_lpcs_translation_; }

auto StoredCalibrationResult::ResidualStandardError() const -> double { return residual_standard_error_; }

void StoredCalibrationResult::SetWeldObjectRotationAxis(const common::Vector3D& value) {
  weld_object_rotation_axis_ = value;
}

void StoredCalibrationResult::SetRotationCenter(const common::Vector3D& value) { rotation_center_ = value; }

void StoredCalibrationResult::SetTorchToLpcsTranslation(const common::Vector3D& value) {
  torch_to_lpcs_translation_ = value;
}
void StoredCalibrationResult::SetResidualStandardError(double value) { residual_standard_error_ = value; }

auto StoredCalibrationResult::ToString() const -> std::string {
  return fmt::format("rotation_axis: {}, rotation_center: {}, torch_to_lpcs: {}, residual_std_err: {:.6f}",
                     common::ToString(weld_object_rotation_axis_), common::ToString(rotation_center_),
                     common::ToString(torch_to_lpcs_translation_), residual_standard_error_);
}

auto StoredCalibrationResult::ToJson() const -> nlohmann::json {
  return {
      {"weldObjectRotationAxis", common::ToJson(weld_object_rotation_axis_)},
      {"rotationCenter",         common::ToJson(rotation_center_)          },
      {"torchToLpcsTranslation", common::ToJson(torch_to_lpcs_translation_)},
      {"residualStandardError",  residual_standard_error_                  }
  };
}

auto StoredCalibrationResult::FromJson(const nlohmann::json& json_obj) -> std::optional<StoredCalibrationResult> {
  StoredCalibrationResult result;

  try {
    auto rot_axis    = common::FromJson<common::Vector3D>(json_obj.at("weldObjectRotationAxis"));
    auto rot_center  = common::FromJson<common::Vector3D>(json_obj.at("rotationCenter"));
    auto translation = common::FromJson<common::Vector3D>(json_obj.at("torchToLpcsTranslation"));

    if (!rot_axis || !rot_center || !translation) {
      LOG_ERROR("Failed to parse one or more Vector3D fields in StoredCalibrationResult::FromJson");
      return std::nullopt;
    }

    result.weld_object_rotation_axis_ = *rot_axis;
    result.rotation_center_           = *rot_center;
    result.torch_to_lpcs_translation_ = *translation;

    if (json_obj.contains("residualStandardError")) {
      result.residual_standard_error_ = json_obj.at("residualStandardError").get<double>();
    }

  } catch (const nlohmann::json::exception& e) {
    LOG_ERROR("Failed to parse StoredCalibrationResult from JSON - exception: {}", e.what());
    return std::nullopt;
  }

  return result;
}

auto StoredCalibrationResult::FromCalibrationResult(const calibration::CalibrationResult& data)
    -> StoredCalibrationResult {
  StoredCalibrationResult result;

  result.SetWeldObjectRotationAxis(data.weld_object_rotation_axis);
  result.SetRotationCenter(data.rotation_center);
  result.SetTorchToLpcsTranslation(data.torch_to_lpcs_translation);
  result.SetResidualStandardError(data.residual_standard_error);
  return result;
}

auto StoredCalibrationResult::IsValid() const -> bool {
  auto is_finite = [](const common::Vector3D& v) {
    return std::isfinite(v.c1) && std::isfinite(v.c2) && std::isfinite(v.c3);
  };

  return is_finite(weld_object_rotation_axis_) && is_finite(rotation_center_) &&
         is_finite(torch_to_lpcs_translation_) && (residual_standard_error_ >= 0.0);
}

void StoredCalibrationResult::CreateTable(SQLite::Database* db) {
  if (db->tableExists(CALIBRATION_RESULT_TABLE_NAME)) {
    return;
  }

  std::string cmd = fmt::format(
      "CREATE TABLE {} ("
      "id INTEGER PRIMARY KEY, "
      "weld_object_rotation_axis TEXT, "
      "rotation_center TEXT, "
      "torch_to_lpcs_translation TEXT, "
      "residual_standard_error REAL)",
      CALIBRATION_RESULT_TABLE_NAME);

  db->exec(cmd);
}

// In this version the Vector3D parameters are stored as individual json objects.
auto StoredCalibrationResult::StoreFn() -> std::function<bool(SQLite::Database*, const StoredCalibrationResult&)> {
  return [](SQLite::Database* db, const StoredCalibrationResult& result) -> bool {
    LOG_TRACE("Store StoredCalibrationResult {}", result.ToString());

    try {
      std::string cmd = fmt::format("INSERT OR REPLACE INTO {} VALUES (1, ?, ?, ?, ?)", CALIBRATION_RESULT_TABLE_NAME);

      SQLite::Statement query(*db, cmd);
      SQLite::bind(query, common::ToJson(result.weld_object_rotation_axis_).dump(),
                   common::ToJson(result.rotation_center_).dump(),
                   common::ToJson(result.torch_to_lpcs_translation_).dump(), result.residual_standard_error_);

      return query.exec() == 1;
    } catch (const std::exception& e) {
      LOG_ERROR("Failed to store StoredCalibrationResult - exception: {}", e.what());
      return false;
    }
  };
}

auto StoredCalibrationResult::GetFn() -> std::function<std::optional<StoredCalibrationResult>(SQLite::Database*)> {
  return [](SQLite::Database* db) -> std::optional<StoredCalibrationResult> {
    std::string cmd = fmt::format("SELECT * FROM {}", CALIBRATION_RESULT_TABLE_NAME);
    SQLite::Statement query(*db, cmd);

    if (!query.executeStep()) {
      return std::nullopt;
    }

    StoredCalibrationResult result;

    auto rot_axis_json    = nlohmann::json::parse(query.getColumn(1).getString());
    auto rot_center_json  = nlohmann::json::parse(query.getColumn(2).getString());
    auto translation_json = nlohmann::json::parse(query.getColumn(3).getString());

    auto rot_axis    = common::FromJson<common::Vector3D>(rot_axis_json);
    auto rot_center  = common::FromJson<common::Vector3D>(rot_center_json);
    auto translation = common::FromJson<common::Vector3D>(translation_json);

    if (!rot_axis || !rot_center || !translation) {
      LOG_ERROR("Failed to parse Vector3D field in StoredCalibrationResult::GetFn");
      return std::nullopt;
    }

    result.weld_object_rotation_axis_ = *rot_axis;
    result.rotation_center_           = *rot_center;
    result.torch_to_lpcs_translation_ = *translation;
    result.residual_standard_error_   = query.getColumn(4).getDouble();

    return result;
  };
}

auto StoredCalibrationResult::ClearFn() -> std::function<bool(SQLite::Database*)> {
  return [](SQLite::Database* db) -> bool {
    try {
      std::string cmd = fmt::format("DELETE FROM {}", CALIBRATION_RESULT_TABLE_NAME);
      return db->exec(cmd) >= 0;
    } catch (const std::exception& e) {
      LOG_ERROR("Failed to clear StoredCalibrationResult - exception: {}", e.what());
      return false;
    }
  };
}

}  // namespace calibration
