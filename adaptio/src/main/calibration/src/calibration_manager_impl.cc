#include "calibration_manager_impl.h"

#include <prometheus/registry.h>
#include <SQLiteCpp/Database.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <functional>
#include <memory>
#include <nlohmann/json_fwd.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "calibration/calibration_configuration.h"
#include "calibration/src/calibration_grid_generator.h"
#include "calibration/src/calibration_sequence_runner.h"
#include "calibration/src/stored_calibration_result.h"
#include "calibration/src/stored_laser_torch_configuration.h"
#include "calibration_log_helpers.h"
#include "calibration_solver.h"
#include "common/clock_functions.h"
#include "common/groove/point.h"
#include "common/logging/application_log.h"
#include "common/logging/component_logger.h"
#include "common/time/format.h"
#include "common/zevs/zevs_socket.h"
#include "coordination/activity_status.h"
#include "joint_geometry/joint_geometry_provider.h"
#include "kinematics/kinematics_client.h"
#include "lpcs/lpcs_slice.h"
#include "scanner_client/scanner_client.h"
#include "slice_translator/model_config.h"
#include "web_hmi/web_hmi.h"

namespace {

const double RSE_LIMIT                      = 1.0;  // Residual standard error limit for successful calibration
const uint32_t CONFIDENCE_DATA_MAX_WAIT_SEC = 3;    // Wait time for data with confidence level > NO

const auto SUCCESS_PAYLOAD = nlohmann::json{
    {"result", "ok"}
};
const auto FAILURE_PAYLOAD = nlohmann::json{
    {"result", "fail"}
};

struct CalibrationStartParams {
  double wire_diameter;
  double stickout;
  double weld_object_radius;
};

auto ParseCalibrationStartPayload(const nlohmann::json& payload) -> std::optional<CalibrationStartParams> {
  try {
    CalibrationStartParams params{};
    payload.at("wireDiameter").get_to(params.wire_diameter);
    payload.at("stickout").get_to(params.stickout);
    payload.at("weldObjectRadius").get_to(params.weld_object_radius);

    if (params.wire_diameter > 0 && params.stickout > 0 && params.weld_object_radius > 0) {
      return params;
    }
    LOG_ERROR("WeldObjectCalStart has invalid numeric values: {}, {}, {}", params.wire_diameter, params.stickout,
              params.weld_object_radius);

  } catch (const nlohmann::json::exception& e) {
    LOG_ERROR("Failed to parse WeldObjectCalStart payload: {}", e.what());
  }

  return std::nullopt;
}
}  // namespace

using calibration::CalibrationManagerImpl;

auto CalibrationManagerImpl::CalibrationContext::TouchDepth() const -> double {
  return top_center.vertical - (left.slide_position.vertical - stickout);
}

CalibrationManagerImpl::CalibrationManagerImpl(
    SQLite::Database* db, zevs::Timer* timer, scanner_client::ScannerClient* scanner_client,
    CalibrationSolver* calibration_solver, slice_translator::ModelConfig* model_config,
    coordination::ActivityStatus* activity_status, web_hmi::WebHmi* web_hmi,
    joint_geometry::JointGeometryProvider* joint_geometry_provider,
    clock_functions::SystemClockNowFunc system_clock_now_func, kinematics::KinematicsClient* kinematics_client,
    prometheus::Registry* /*registry*/, const std::filesystem::path& path_logs, const GridConfiguration& grid_config,
    const RunnerConfiguration& runner_config)
    : calibration_result_storage_(db, StoredCalibrationResult::CreateTable, StoredCalibrationResult::StoreFn(),
                                  StoredCalibrationResult::GetFn()),
      laser_torch_configuration_storage_(db, StoredLaserTorchConfiguration::CreateTable,
                                         StoredLaserTorchConfiguration::StoreFn(),
                                         StoredLaserTorchConfiguration::GetFn()),
      timer_(timer),
      scanner_client_(scanner_client),
      calibration_solver_(calibration_solver),
      model_config_(model_config),
      activity_status_(activity_status),
      web_hmi_(web_hmi),
      joint_geometry_provider_(joint_geometry_provider),
      kinematics_client_(kinematics_client),
      system_clock_now_func_(system_clock_now_func),
      grid_config_(grid_config),
      runner_config_(runner_config)

{
  scanner_client_->AddObserver(this);
  SubscribeWebHmi();

  LOG_INFO("Calibration grid_config: {}", ToJsonLog(grid_config_).dump());
  LOG_INFO("Calibration runner_config: {}", ToJsonLog(runner_config_).dump());

  const auto stored_laser_torch_configuration = laser_torch_configuration_storage_.Get();
  if (stored_laser_torch_configuration) {
    LOG_INFO("Laser to torch configuration: {}", stored_laser_torch_configuration->ToString());
  }

  const auto stored_calibration_result = calibration_result_storage_.Get();
  if (stored_calibration_result) {
    LOG_INFO("Calibration result: {}", stored_calibration_result->ToString());
  }

  if (stored_laser_torch_configuration.has_value() && stored_calibration_result.has_value()) {
    std::array<double, 3> scanner_angles = {stored_laser_torch_configuration.value().ScannerMountAngle(), 0.0, 0.0};

    model_config_->Set(stored_calibration_result.value().RotationCenter(), scanner_angles,
                       stored_calibration_result.value().WeldObjectRotationAxis(),
                       stored_calibration_result.value().TorchToLpcsTranslation(),
                       stored_calibration_result->WeldObjectRadius());
  }

  auto const cl_config = common::logging::ComponentLoggerConfig{
      .component      = "calibration",
      .path_directory = path_logs / "calibration",
      .file_name      = "%Y%m%d_%H%M%S.log",
      .max_file_size  = 1 * 1000 * 1000, /* 1 MB */
      .max_nb_files   = 100,
  };

  calibration_logger_ = common::logging::ComponentLogger(cl_config);
}

// ScannerObserver
void CalibrationManagerImpl::OnScannerStarted(bool success) {
  if (calibration_start_procedure_) {
    if (success) {
      LOG_INFO("Scanner started for calibration");
    } else {
      LOG_ERROR("Scanner start failed");
    }

    (*calibration_start_procedure_)(success);
  }
};

void CalibrationManagerImpl::OnScannerStopped(bool /*success*/) { /*do nothing*/ };

void CalibrationManagerImpl::OnScannerDataUpdate(const lpcs::Slice& data, const common::Point& axis_position) {
  if (sequence_runner_ && sequence_runner_->Busy()) {
    sequence_runner_->OnScannerDataUpdate(data, axis_position);
    return;
  }

  // Convert procedure_start_time to raw ticks for comparison with data.time_stamp (uint64_t)
  if (data.time_stamp < procedure_start_time_.time_since_epoch().count()) {
    return;
  }

  if (calibration_top_pos_procedure_) {
    HandleTopPosData(axis_position);
    return;
  }

  if (calibration_left_pos_procedure_) {
    HandleLeftPosData(data, axis_position);
    return;
  }

  if (calibration_right_pos_procedure_) {
    HandleRightPosData(data, axis_position);
    return;
  }
}

void CalibrationManagerImpl::HandleTopPosData(const common::Point& axis_position) {
  if (CheckProcedureExpired()) {
    HandleTopTouchFailure("Procedure expired");
    return;
  }

  (*calibration_top_pos_procedure_)(axis_position);
}

void CalibrationManagerImpl::HandleLeftPosData(const lpcs::Slice& data, const common::Point& axis_position) {
  if (CheckProcedureExpired()) {
    HandleLeftTouchFailure("Procedure expired (no scanner data)");
    return;
  }

  if (data.confidence == lpcs::SliceConfidence::NO || !data.groove) {
    return;
  }

  Observation observation{.slide_position = axis_position, .abw_points_lpcs = *data.groove};
  (*calibration_left_pos_procedure_)(observation);
}

void CalibrationManagerImpl::HandleRightPosData(const lpcs::Slice& data, const common::Point& axis_position) {
  if (CheckProcedureExpired()) {
    HandleRightTouchFailure("Procedure expired (no scanner data)");
    return;
  }

  if (data.confidence == lpcs::SliceConfidence::NO || !data.groove) {
    return;
  }

  Observation observation{.slide_position = axis_position, .abw_points_lpcs = *data.groove};
  (*calibration_right_pos_procedure_)(observation);
}

// coordination::CalibrationStatus
auto CalibrationManagerImpl::WeldObjectCalibrationValid() const -> bool {
  // Consider weld-object calibration valid only if both LTC and WO calibration are stored and model is configured
  const auto ltc = laser_torch_configuration_storage_.Get();
  const auto woc = calibration_result_storage_.Get();
  return ltc.has_value() && woc.has_value();
};
void CalibrationManagerImpl::Subscribe(std::function<void()> subscriber) {
  calibration_status_subscriber_ = subscriber;
};

void CalibrationManagerImpl::OnTimeout() {}

auto CalibrationManagerImpl::Busy() const -> bool {
  return calibration_start_procedure_.has_value() || calibration_top_pos_procedure_.has_value() ||
         calibration_left_pos_procedure_.has_value() || calibration_right_pos_procedure_.has_value() ||
         sequence_runner_;
}

void CalibrationManagerImpl::SetProcedureStartTime() { procedure_start_time_ = system_clock_now_func_(); }

auto CalibrationManagerImpl::CheckProcedureExpired() -> bool {
  const auto now      = system_clock_now_func_();
  const auto deadline = procedure_start_time_ + std::chrono::seconds(CONFIDENCE_DATA_MAX_WAIT_SEC);

  return now > deadline;
}

void CalibrationManagerImpl::SubscribeWebHmi() {
  web_hmi_->Subscribe("LaserTorchCalGet",
                      [this](std::string const&, const nlohmann::json&) { this->OnLaserTorchCalGet(); });

  web_hmi_->Subscribe("LaserTorchCalSet",
                      [this](std::string const&, const nlohmann::json& payload) { this->OnLaserTorchCalSet(payload); });

  web_hmi_->Subscribe("WeldObjectCalGet",
                      [this](std::string const&, const nlohmann::json&) { this->OnWeldObjectCalGet(); });

  web_hmi_->Subscribe("WeldObjectCalSet",
                      [this](std::string const&, const nlohmann::json& payload) { this->OnWeldObjectCalSet(payload); });

  web_hmi_->Subscribe("WeldObjectCalStart", [this](std::string const&, const nlohmann::json& payload) {
    this->OnWeldObjectCalStart(payload);
  });

  web_hmi_->Subscribe("WeldObjectCalStop",
                      [this](std::string const&, const nlohmann::json&) { this->OnWeldObjectCalStop(); });

  web_hmi_->Subscribe("WeldObjectCalTopPos",
                      [this](std::string const&, const nlohmann::json&) { this->OnWeldObjectCalTopPos(); });

  web_hmi_->Subscribe("WeldObjectCalLeftPos",
                      [this](std::string const&, const nlohmann::json&) { this->OnWeldObjectCalLeftPos(); });

  web_hmi_->Subscribe("WeldObjectCalRightPos",
                      [this](std::string const&, const nlohmann::json&) { this->OnWeldObjectCalRightPos(); });
}

void CalibrationManagerImpl::OnLaserTorchCalGet() {
  if (const auto laser_torch_configuration = laser_torch_configuration_storage_.Get()) {
    nlohmann::json payload = laser_torch_configuration.value().ToJson();
    web_hmi_->Send("LaserTorchCalGetRsp", SUCCESS_PAYLOAD, payload);
  } else {
    web_hmi_->Send("LaserTorchCalGetRsp", FAILURE_PAYLOAD, "No valid laser to torch calibration stored", std::nullopt);
  }
}

void CalibrationManagerImpl::OnLaserTorchCalSet(const nlohmann::json& payload) {
  if (activity_status_->Get() == coordination::ActivityStatusE::WELD_OBJECT_CALIBRATION) {
    StopCalibration();
  }

  const auto cal_data = StoredLaserTorchConfiguration::FromJson(payload);
  if (cal_data.has_value()) {
    auto result = laser_torch_configuration_storage_.Store(cal_data.value());
    if (result) {
      if (const auto stored = laser_torch_configuration_storage_.Get()) {
        LOG_INFO("Stored laser torch configuration updated to: {}", stored->ToJson().dump());
      }

      calibration_result_storage_.Clear();
      model_config_->Reset();
      web_hmi_->Send("LaserTorchCalSetRsp", SUCCESS_PAYLOAD, std::nullopt);
      return;
    }
  }

  web_hmi_->Send("LaserTorchCalSetRsp", FAILURE_PAYLOAD, "Unable to store laser to torch calibration data",
                 std::nullopt);
}

void CalibrationManagerImpl::OnWeldObjectCalGet() {
  if (const auto calibration_result = calibration_result_storage_.Get()) {
    nlohmann::json payload = calibration_result.value().ToJson();
    web_hmi_->Send("WeldObjectCalGetRsp", SUCCESS_PAYLOAD, payload);
  } else {
    web_hmi_->Send("WeldObjectCalGetRsp", FAILURE_PAYLOAD, "No valid weld-object calibration stored", std::nullopt);
  }
}

void CalibrationManagerImpl::OnWeldObjectCalSet(const nlohmann::json& payload) {
  const auto cal_data                  = StoredCalibrationResult::FromJson(payload);
  const auto laser_torch_configuration = laser_torch_configuration_storage_.Get();
  if (cal_data.has_value() && laser_torch_configuration.has_value()) {
    auto result = calibration_result_storage_.Store(cal_data.value());
    if (result) {
      if (const auto stored = calibration_result_storage_.Get()) {
        LOG_INFO("Stored calibration result updated to: {}", stored->ToJson().dump());
      }
      std::array<double, 3> scanner_angles = {laser_torch_configuration.value().ScannerMountAngle(), 0.0, 0.0};

      model_config_->Set(cal_data.value().RotationCenter(), scanner_angles, cal_data.value().WeldObjectRotationAxis(),
                         cal_data.value().TorchToLpcsTranslation(), cal_data.value().WeldObjectRadius());
      web_hmi_->Send("WeldObjectCalSetRsp", SUCCESS_PAYLOAD, std::nullopt);
      calibration_status_subscriber_();

      return;
    }
  }

  web_hmi_->Send("WeldObjectCalSetRsp", FAILURE_PAYLOAD, "Unable to store weld-object calibration data", std::nullopt);
}

void CalibrationManagerImpl::OnWeldObjectCalStart(const nlohmann::json& payload) {
  if (!laser_torch_configuration_storage_.Get()) {
    SendCalibrationStartFailure("Laser to torch calibration missing");
    return;
  }

  if (Busy()) {
    SendCalibrationStartFailure("Calibration already in progress");
    return;
  }

  if (!activity_status_->IsIdle()) {
    SendCalibrationStartFailure("Activity status not idle");
    return;
  }

  auto params = ParseCalibrationStartPayload(payload);
  if (!params) {
    SendCalibrationStartFailure("Invalid or missing calibration parameters in payload");
    return;
  }

  LOG_INFO("WeldObjectCalStart: wireDiameter={}, stickout={}, weldObjectRadius={}", params->wire_diameter,
           params->stickout, params->weld_object_radius);

  auto joint_geometry = joint_geometry_provider_->GetJointGeometry();
  if (!joint_geometry) {
    SendCalibrationStartFailure("Joint geometry not available");
    return;
  }

  calibration_ctx_.wire_diameter      = params->wire_diameter;
  calibration_ctx_.stickout           = params->stickout;
  calibration_ctx_.weld_object_radius = params->weld_object_radius;
  calibration_ctx_.laser_torch_config = laser_torch_configuration_storage_.Get().value();
  calibration_ctx_.joint_geometry     = joint_geometry.value();

  activity_status_->Set(coordination::ActivityStatusE::WELD_OBJECT_CALIBRATION);

  calibration_start_procedure_ = [this](bool success) {
    LOG_INFO("Calibration start procedure completed ");
    web_hmi_->Send("WeldObjectCalStartRsp", success ? SUCCESS_PAYLOAD : FAILURE_PAYLOAD,
                   success ? std::nullopt : std::make_optional<std::string>("Scanner start failed"), std::nullopt);
    if (!success) {
      StopCalibration();
    }
    calibration_start_procedure_ = {};
  };

  scanner_client_->Start({}, *joint_geometry);
}

void CalibrationManagerImpl::SendCalibrationStartFailure(const std::string& reason) {
  LOG_ERROR("WeldObjectCalStart failed: {}", reason);
  web_hmi_->Send("WeldObjectCalStartRsp", FAILURE_PAYLOAD, reason, std::nullopt);
}

void CalibrationManagerImpl::OnWeldObjectCalStop() {
  LOG_INFO("WeldObjectCalStop received");
  StopCalibration();
  web_hmi_->Send("WeldObjectCalStopRsp", SUCCESS_PAYLOAD, std::nullopt);
}

void CalibrationManagerImpl::OnWeldObjectCalTopPos() {
  if (Busy()) {
    LOG_INFO("WeldObjectCalTopPos received when busy");
    web_hmi_->Send("WeldObjectCalTopPosRsp", FAILURE_PAYLOAD, "Busy", std::nullopt);
    return;
  }

  if (activity_status_->Get() != coordination::ActivityStatusE::WELD_OBJECT_CALIBRATION) {
    LOG_INFO("WeldObjectCalTopPos received when activity status not WELD_OBJECT_CALIBRATION");
    web_hmi_->Send("WeldObjectCalTopPosRsp", FAILURE_PAYLOAD, "Activity status not WELD_OBJECT_CALIBRATION",
                   std::nullopt);
    return;
  }

  LOG_INFO("WeldObjectCalTopPos received");
  SetProcedureStartTime();

  calibration_top_pos_procedure_ = [this](const common::Point& axis_position) {
    this->OnTopPosProcedureComplete(axis_position);
  };
}

void CalibrationManagerImpl::OnTopPosProcedureComplete(const common::Point& axis_position) {
  calibration_ctx_.top = axis_position;
  LOG_INFO("Calibration top touch position, recorded at h: {:.2f}, v: {:.2f}", axis_position.horizontal,
           axis_position.vertical);
  web_hmi_->Send("WeldObjectCalTopPosRsp", SUCCESS_PAYLOAD, std::nullopt);
  calibration_top_pos_procedure_ = {};
}

void CalibrationManagerImpl::HandleTopTouchFailure(const std::string& reason) {
  LOG_ERROR("Calibration top touch procedure failed: {}", reason);
  web_hmi_->Send("WeldObjectCalTopPosRsp", FAILURE_PAYLOAD, reason, std::nullopt);
  StopCalibration();
}

void CalibrationManagerImpl::OnWeldObjectCalLeftPos() {
  if (Busy()) {
    LOG_INFO("WeldObjectCalLeftPos received when busy");
    web_hmi_->Send("WeldObjectCalLeftPosRsp", FAILURE_PAYLOAD, "Busy", std::nullopt);
    return;
  }

  if (activity_status_->Get() != coordination::ActivityStatusE::WELD_OBJECT_CALIBRATION) {
    LOG_INFO("WeldObjectCalLeftPos received when activity status not WELD_OBJECT_CALIBRATION");
    web_hmi_->Send("WeldObjectCalLeftPosRsp", FAILURE_PAYLOAD, "Activity status not WELD_OBJECT_CALIBRATION",
                   std::nullopt);
    return;
  }

  LOG_INFO("WeldObjectCalLeftPos received");
  SetProcedureStartTime();

  calibration_left_pos_procedure_ = [this](const std::optional<Observation>& observation) {
    this->OnLeftPosProcedureComplete(observation);
  };
}

void CalibrationManagerImpl::OnLeftPosProcedureComplete(const std::optional<Observation>& observation) {
  if (!observation) {
    HandleLeftTouchFailure("observation failed");
    return;
  }

  const auto& obs       = observation.value();
  calibration_ctx_.left = obs;
  LOG_INFO("Calibration left touch position, recorded at h: {:.2f}, v: {:.2f}", obs.slide_position.horizontal,
           obs.slide_position.vertical);
  web_hmi_->Send("WeldObjectCalLeftPosRsp", SUCCESS_PAYLOAD, std::nullopt);
  calibration_left_pos_procedure_ = {};
}

void CalibrationManagerImpl::HandleLeftTouchFailure(const std::string& reason) {
  LOG_ERROR("Calibration left touch procedure failed: {}", reason);
  web_hmi_->Send("WeldObjectCalLeftPosRsp", FAILURE_PAYLOAD, reason, std::nullopt);
  StopCalibration();
}

void CalibrationManagerImpl::StopCalibration() {
  LOG_INFO("Stopping calibration procedure");
  scanner_client_->Stop();
  calibration_ctx_                 = CalibrationContext{};
  calibration_top_pos_procedure_   = {};
  calibration_left_pos_procedure_  = {};
  calibration_right_pos_procedure_ = {};
  sequence_runner_                 = {};
  activity_status_->Set(coordination::ActivityStatusE::IDLE);
}

void CalibrationManagerImpl::OnWeldObjectCalRightPos() {
  if (Busy()) {
    LOG_INFO("WeldObjectCalRightPos received when busy");
    web_hmi_->Send("WeldObjectCalRightPosRsp", FAILURE_PAYLOAD, "Busy", std::nullopt);
    return;
  }

  if (activity_status_->Get() != coordination::ActivityStatusE::WELD_OBJECT_CALIBRATION) {
    LOG_INFO("WeldObjectCalRightPos received when activity status not WELD_OBJECT_CALIBRATION");
    web_hmi_->Send("WeldObjectCalRightPosRsp", FAILURE_PAYLOAD, "Activity status not WELD_OBJECT_CALIBRATION",
                   std::nullopt);
    return;
  }

  LOG_INFO("WeldObjectCalRightPos received");
  SetProcedureStartTime();

  // Create the procedure, then wait for scannerdata with timestamp in the future
  calibration_right_pos_procedure_ = [this](const std::optional<Observation>& observation) {
    this->OnRightPosProcedureComplete(observation);
  };
}

void CalibrationManagerImpl::OnRightPosProcedureComplete(const std::optional<Observation>& observation) {
  if (!observation) {
    HandleRightTouchFailure("observation failed");
    return;
  }

  const auto& obs        = observation.value();
  calibration_ctx_.right = obs;
  LOG_INFO("Calibration right touch position, recorded at h: {:.2f}, v: {:.2f}", obs.slide_position.horizontal,
           obs.slide_position.vertical);

  const auto top_center = CalculateTopCenter();
  if (!top_center) {
    HandleRightTouchFailure("top_center calculation failed");
    return;
  }

  calibration_ctx_.top_center = top_center.value();

  if (calibration_ctx_.left.slide_position.horizontal < calibration_ctx_.right.slide_position.horizontal) {
    LOG_INFO("Walls touched in wrong order");
    // Check if wall is touched in wrong order. Allow it and swap to the correct positions
    std::swap(calibration_ctx_.left, calibration_ctx_.right);
  }

  // At this point the right-pos is considered ok
  LOG_INFO("Top center calculated to h: {:.2f}, v: {:.2f}", top_center.value().horizontal, top_center.value().vertical);
  web_hmi_->Send("WeldObjectCalRightPosRsp", SUCCESS_PAYLOAD, std::nullopt);

  const auto grid_points = GenerateGridPoints();
  StartCalibrationSequence(grid_points);
}

void CalibrationManagerImpl::HandleRightTouchFailure(const std::string& reason) {
  LOG_ERROR("Calibration right touch procedure failed: {}", reason);
  web_hmi_->Send("WeldObjectCalRightPosRsp", FAILURE_PAYLOAD, reason, std::nullopt);
  StopCalibration();
}

auto CalibrationManagerImpl::CalculateTopCenter() -> std::optional<common::Point> {
  const auto& joint_geometry = calibration_ctx_.joint_geometry;
  const auto& top_obs        = calibration_ctx_.top;
  const auto& left_obs       = calibration_ctx_.left;
  const auto& right_obs      = calibration_ctx_.right;
  const auto stickout        = calibration_ctx_.stickout;
  const auto wire_diameter   = calibration_ctx_.wire_diameter;

  // support only one top center point methods
  if (!calibration_ctx_.top) {
    LOG_ERROR("No top observation available");
    return {};
  }

  common::Point top_point = top_obs.value();
  return ValidateAndCalculateGrooveTopCenter2(joint_geometry, wire_diameter, stickout, left_obs.slide_position,
                                              right_obs.slide_position, top_point);
}

auto CalibrationManagerImpl::GenerateGridPoints() -> std::vector<GridPoint> {
  const auto& left_pos  = calibration_ctx_.left.slide_position;
  const auto& right_pos = calibration_ctx_.right.slide_position;

  auto touch_depth = calibration_ctx_.TouchDepth();
  LOG_INFO("Touch depth calculated to {:.1f} mm", touch_depth);

  return GenerateCalibrationDots(grid_config_, left_pos.horizontal, right_pos.horizontal, right_pos.vertical,
                                 touch_depth);
}

void CalibrationManagerImpl::StartCalibrationSequence(std::vector<GridPoint> grid_points) {
  sequence_runner_ = std::make_unique<CalibrationSequenceRunner>(
      timer_, kinematics_client_, std::move(grid_points),

      // On failure
      [this]() {
        LOG_ERROR("Calibration sequence failed");
        web_hmi_->Send("WeldObjectCalResult", FAILURE_PAYLOAD, "Calibration sequence failed", std::nullopt);
        StopCalibration();
        kinematics_client_->Release();
      },

      // On complete
      [this](const std::vector<Observation>& observations) { this->OnCalibrationSequenceComplete(observations); },

      // On progress
      [this](double progress) {
        LOG_DEBUG("Calibration progress: {}%", std::round(progress * 100));
        web_hmi_->Send("WeldObjectCalProgress", std::nullopt,
                       nlohmann::json{
                           {"progress", progress}
        });
      },

      runner_config_);

  sequence_runner_->Start();

  activity_status_->Set(coordination::ActivityStatusE::CALIBRATION_AUTO_MOVE);
}

void CalibrationManagerImpl::OnCalibrationSequenceComplete(const std::vector<Observation>& observations) {
  LOG_INFO("Calibration measurement sequence complete");
  const TorchPlaneInfo torch_plane_info{.top_center_at_torch_plane = calibration_ctx_.top_center};

  const auto& ltc = calibration_ctx_.laser_torch_config;
  const GeometricConstants geometric_constants{.object_radius            = calibration_ctx_.weld_object_radius,
                                               .scanner_mount_angle      = ltc.ScannerMountAngle(),
                                               .ltc_stickout             = ltc.Stickout(),
                                               .ltc_laser_plane_distance = ltc.DistanceLaserTorch()};

  const auto result = TryComputeCalibrationResult(torch_plane_info, geometric_constants, observations);

  ReportCalibrationResult(result, torch_plane_info, geometric_constants, observations);

  StopCalibration();
  kinematics_client_->Release();
}

auto CalibrationManagerImpl::TryComputeCalibrationResult(const TorchPlaneInfo& torch_plane_info,
                                                         const GeometricConstants& geometric_constants,
                                                         const std::vector<Observation>& observations)
    -> std::optional<CalibrationResult> {
  try {
    LOG_INFO("Calling calculate with: torch_plane_info={}, geometric_constants={}", ToString(torch_plane_info),
             ToString(geometric_constants));
    return calibration_solver_->Calculate(torch_plane_info, geometric_constants, observations);
  } catch (const std::exception& e) {
    LOG_ERROR("CalibrationSolver::Calculate threw: {}", e.what());
    return std::nullopt;
  }
}

void CalibrationManagerImpl::ReportCalibrationResult(const std::optional<CalibrationResult>& result,
                                                     const TorchPlaneInfo& torch_plane_info,
                                                     const GeometricConstants& geometric_constants,
                                                     const std::vector<Observation>& observations) {
  if (result.has_value()) {
    auto calibration_result =
        StoredCalibrationResult::FromCalibrationResult(result.value(), calibration_ctx_.weld_object_radius);
    nlohmann::json payload = calibration_result.ToJson();
    if (result->residual_standard_error < RSE_LIMIT) {
      LOG_INFO("Successful calibration result: {}", payload.dump());
      web_hmi_->Send("WeldObjectCalResult", SUCCESS_PAYLOAD, payload);
    } else {
      LOG_ERROR("RSE too high in calibration: {}", payload.dump());
      web_hmi_->Send("WeldObjectCalResult", FAILURE_PAYLOAD, "Residual standard error too high", std::nullopt);
    }
  } else {
    LOG_ERROR("Calibration failed");
    web_hmi_->Send("WeldObjectCalResult", FAILURE_PAYLOAD, "Calibration failed", std::nullopt);
  }

  LogCalibrationRun(torch_plane_info, geometric_constants, observations, result);
}

void CalibrationManagerImpl::LogCalibrationRun(const TorchPlaneInfo& tp, const GeometricConstants& gc,
                                               const std::vector<Observation>& obs,
                                               const std::optional<CalibrationResult>& result) {
  auto now           = system_clock_now_func_();
  nlohmann::json log = {
      {"timestamp", common::time::TimePointToString(now, common::time::FMT_TS_MS)},
      {"torchPlaneInfo", ToJsonLog(tp)},
      {"geometricConstants", ToJsonLog(gc)},
      {"grid_config", ToJsonLog(grid_config_)},
      {"runner_config", ToJsonLog(runner_config_)}
  };

  log["jointGeometry"] = ToJsonLog(calibration_ctx_.joint_geometry);
  if (calibration_ctx_.top) {
    log["topPosObservation"] = ToJsonLog(*calibration_ctx_.top);
  }
  log["leftPosObservation"]  = ToJsonLog(calibration_ctx_.left);
  log["rightPosObservation"] = ToJsonLog(calibration_ctx_.right);

  for (const auto& item : obs) {
    log["observations"].push_back(ToJsonLog(item));
  }

  if (result) {
    log["calibrationResult"] = ToJsonLog(*result);
  }

  calibration_logger_.Log(log.dump());
}
