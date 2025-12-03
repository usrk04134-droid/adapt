#pragma once

#include <prometheus/registry.h>

#include "calibration_handler.h"
#include "calibration_solver.h"
#include "common/clock_functions.h"
#include "common/zevs/zevs_socket.h"
#include "coordination/activity_status.h"
#include "coordination/calibration_status.h"
#include "cw_calibration_handler.h"
#include "joint_geometry/joint_geometry_provider.h"
#include "kinematics/kinematics_client.h"
#include "lw_calibration_handler.h"
#include "scanner_client/scanner_client.h"
#include "slice_translator/model_config.h"
#include "web_hmi/web_hmi.h"

namespace calibration {

class CalibrationManagerImpl : public scanner_client::ScannerObserver, public coordination::CalibrationStatus {
 public:
  CalibrationManagerImpl(SQLite::Database* db, zevs::Timer* timer, scanner_client::ScannerClient* scanner_client,
                         CalibrationSolver* calibration_solver, slice_translator::ModelConfig* model_config,
                         coordination::ActivityStatus* activity_status, web_hmi::WebHmi* web_hmi,
                         joint_geometry::JointGeometryProvider* joint_geometry_provider,
                         clock_functions::SystemClockNowFunc system_clock_now_func,
                         kinematics::KinematicsClient* kinematics_client, prometheus::Registry* registry,
                         const std::filesystem::path& path_logs, const GridConfiguration& grid_config,
                         const RunnerConfiguration& runner_config);

  void OnScannerStarted(bool success) override;
  void OnScannerStopped(bool success) override;
  void OnScannerDataUpdate(const lpcs::Slice& data, const common::Point& axis_position) override;

  auto WeldObjectCalibrationValid() const -> bool override;
  void Subscribe(std::function<void()> subscriber) override;

 private:
  auto GetActiveHandler() -> CalibrationHandler*;

  DependencyContext handler_context_;
  common::logging::ComponentLogger calibration_logger_;
  std::unique_ptr<CWCalibrationHandler> cw_handler_;
  std::unique_ptr<LWCalibrationHandler> lw_handler_;
  std::function<void()> calibration_status_subscriber_;
};

}  // namespace calibration
