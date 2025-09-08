#pragma once

#include <prometheus/registry.h>

#include <chrono>
#include <optional>

#include "calibration/calibration_metrics.h"
#include "calibration_sequence_runner.h"
#include "calibration_solver.h"
#include "common/clock_functions.h"
#include "common/logging/component_logger.h"
#include "common/storage/sql_single_storage.h"
#include "common/zevs/zevs_socket.h"
#include "coordination/activity_status.h"
#include "coordination/calibration_status.h"
#include "joint_geometry/joint_geometry_provider.h"
#include "kinematics/kinematics_client.h"
#include "scanner_client/scanner_client.h"
#include "slice_translator/model_config.h"
#include "stored_calibration_result.h"
#include "stored_laser_torch_configuration.h"
#include "web_hmi/web_hmi.h"

namespace calibration {

class CalibrationManagerV2Impl : public scanner_client::ScannerObserver, public coordination::CalibrationStatus {
 public:
  CalibrationManagerV2Impl(SQLite::Database* db, zevs::Timer* timer, scanner_client::ScannerClient* scanner_client,
                           CalibrationSolver* calibration_solver, slice_translator::ModelConfig* model_config,
                           coordination::ActivityStatus* activity_status, web_hmi::WebHmi* web_hmi,
                           joint_geometry::JointGeometryProvider* joint_geometry_provider,
                           clock_functions::SystemClockNowFunc system_clock_now_func,
                           kinematics::KinematicsClient* kinematics_client, prometheus::Registry* registry,
                           const std::filesystem::path& path_logs, const GridConfiguration& grid_config,
                           const RunnerConfiguration& runner_config);

  // ScannerObserver
  void OnScannerStarted(bool success) override;
  void OnScannerStopped(bool success) override;
  void OnScannerDataUpdate(const lpcs::Slice& data, const macs::Point& axis_position) override;

  // coordination::CalibrationStatus
  auto WeldObjectCalibrationValid() const -> bool override;
  void Subscribe(std::function<void()> subscriber) override;

 private:
  void OnTimeout();
  void SubscribeWebHmi();
  void SetProcedureStartTime();
  auto CheckProcedureExpired() -> bool;
  auto Busy() const -> bool;
  void StopCalibration();

  // Message Handlers
  void OnLaserTorchCalGet();
  void OnLaserTorchCalSet(const nlohmann::json& payload);
  void OnWeldObjectCalGet();
  void OnWeldObjectCalSet(const nlohmann::json& payload);
  void OnWeldObjectCalStart(const nlohmann::json& payload);
  void OnWeldObjectCalStop();
  void OnWeldObjectCalTopPos();
  void OnWeldObjectCalLeftPos();
  void OnWeldObjectCalRightPos();

  // Holds all calibration-related data for a single session.
  // This is a regular member (not optional) for simplicity,
  // since the active session state is already managed via activity_status_.
  struct CalibrationContext {
    macs::Point top_center;
    StoredLaserTorchConfiguration laser_torch_config;
    double wire_diameter{};
    double stickout{};
    double weld_object_radius{};
    joint_geometry::JointGeometry joint_geometry;
    std::vector<Observation> observations;

    std::optional<macs::Point> top;
    Observation left;
    Observation right;

    auto TouchDepth() const -> double;
  } calibration_ctx_;

  // Procedure Handlers and helpers
  void SendCalibrationStartFailure(const std::string& reason);
  void HandleTopPosData(const macs::Point& axis_position);
  void HandleLeftPosData(const lpcs::Slice& data, const macs::Point& axis_position);
  void HandleRightPosData(const lpcs::Slice& data, const macs::Point& axis_position);
  void OnTopPosProcedureComplete(const macs::Point& axis_position);
  void HandleTopTouchFailure(const std::string& reason);
  void OnLeftPosProcedureComplete(const std::optional<Observation>& observation);
  void HandleLeftTouchFailure(const std::string& reason);
  void OnRightPosProcedureComplete(const std::optional<Observation>& observation);
  void HandleRightTouchFailure(const std::string& reason);
  auto CalculateTopCenter() -> std::optional<macs::Point>;
  auto GenerateGridPoints() -> std::vector<GridPoint>;
  void StartCalibrationSequence(std::vector<GridPoint> grid_points);
  void OnCalibrationSequenceComplete(const std::vector<Observation>& observations);
  auto TryComputeCalibrationResult(const TorchPlaneInfo& torch_plane_info,
                                   const GeometricConstants& geometric_constants,
                                   const std::vector<Observation>& observations) -> std::optional<CalibrationResult>;
  void ReportCalibrationResult(const std::optional<CalibrationResult>& result, const TorchPlaneInfo& torch_plane_info,
                               const GeometricConstants& geometric_constants,
                               const std::vector<Observation>& observations);
  void LogCalibrationRun(const TorchPlaneInfo& tp, const GeometricConstants& gc, const std::vector<Observation>& obs,
                         const std::optional<CalibrationResult>& result);

  // These members represent ongoing procedures. If any are active (non-empty),
  // Busy() will return true, preventing the start of a new calibration session.
  using StartProcedure = std::optional<std::function<void(bool)>>;
  StartProcedure calibration_start_procedure_;
  using PositionProcedure = std::optional<std::function<void(const macs::Point&)>>;
  PositionProcedure calibration_top_pos_procedure_;
  using ObservationProcedure = std::optional<std::function<void(const std::optional<Observation>&)>>;
  ObservationProcedure calibration_left_pos_procedure_;
  ObservationProcedure calibration_right_pos_procedure_;
  std::unique_ptr<CalibrationSequenceRunner> sequence_runner_;

  storage::SqlSingleStorage<StoredCalibrationResult> calibration_result_storage_;
  storage::SqlSingleStorage<StoredLaserTorchConfiguration> laser_torch_configuration_storage_;
  zevs::Timer* timer_;
  std::optional<uint32_t> timer_task_id_;
  scanner_client::ScannerClient* scanner_client_;
  CalibrationSolver* calibration_solver_;
  slice_translator::ModelConfig* model_config_;
  coordination::ActivityStatus* activity_status_;
  web_hmi::WebHmi* web_hmi_;
  joint_geometry::JointGeometryProvider* joint_geometry_provider_;
  kinematics::KinematicsClient* kinematics_client_;
  clock_functions::SystemClockNowFunc system_clock_now_func_;
  GridConfiguration grid_config_;
  RunnerConfiguration runner_config_;
  common::logging::ComponentLogger calibration_logger_;
  std::function<void()> calibration_status_subscriber_;
  std::chrono::time_point<std::chrono::system_clock> procedure_start_time_;

  std::unique_ptr<CalibrationMetrics> cal_metrics_;
};

}  // namespace calibration
