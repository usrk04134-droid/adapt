#include "calibration_manager_impl.h"

#include <prometheus/registry.h>
#include <SQLiteCpp/Database.h>

#include <functional>
#include <memory>
#include <nlohmann/json.hpp>

#include "calibration/calibration_configuration.h"
#include "common/logging/component_logger.h"
#include "scanner_client/scanner_client.h"
#include "web_hmi/web_hmi.h"

using calibration::CalibrationManagerImpl;

CalibrationManagerImpl::CalibrationManagerImpl(
    SQLite::Database* db, zevs::Timer* timer, scanner_client::ScannerClient* scanner_client,
    CalibrationSolver* calibration_solver, slice_translator::ModelConfig* model_config,
    coordination::ActivityStatus* activity_status, web_hmi::WebHmi* web_hmi,
    joint_geometry::JointGeometryProvider* joint_geometry_provider,
    clock_functions::SystemClockNowFunc system_clock_now_func, kinematics::KinematicsClient* kinematics_client,
    prometheus::Registry* /*registry*/, const std::filesystem::path& path_logs, const GridConfiguration& grid_config,
    const RunnerConfiguration& runner_config)
    : handler_context_{.scanner_client          = scanner_client,
                       .activity_status         = activity_status,
                       .web_hmi                 = web_hmi,
                       .joint_geometry_provider = joint_geometry_provider,
                       .kinematics_client       = kinematics_client,
                       .system_clock_now_func   = system_clock_now_func,
                       .calibration_logger      = nullptr},
      calibration_logger_(common::logging::ComponentLoggerConfig{
          .component      = "calibration",
          .path_directory = path_logs / "calibration",
          .file_name      = "%Y%m%d_%H%M%S.log",
          .max_file_size  = 1 * 1000 * 1000, /* 1 MB */
          .max_nb_files   = 100,
      }),
      cw_handler_(std::make_unique<CWCalibrationHandler>(db, &handler_context_, calibration_solver, model_config, timer,
                                                         grid_config, runner_config,
                                                         [this]() {
                                                           if (calibration_status_subscriber_) {
                                                             calibration_status_subscriber_();
                                                           }
                                                         })),
      lw_handler_(std::make_unique<LWCalibrationHandler>(db, &handler_context_, [this]() {
        if (calibration_status_subscriber_) {
          calibration_status_subscriber_();
        }
      })) {
  handler_context_.calibration_logger = &calibration_logger_;
  scanner_client->AddObserver(this);

  cw_handler_->SubscribeWebHmi();
  lw_handler_->SubscribeWebHmi();
}

void CalibrationManagerImpl::OnScannerStarted(bool success) {
  if (auto* handler = GetActiveHandler()) {
    handler->OnScannerStarted(success);
  }
}

void CalibrationManagerImpl::OnScannerStopped(bool success) {
  if (auto* handler = GetActiveHandler()) {
    handler->OnScannerStopped(success);
  }
}

void CalibrationManagerImpl::OnScannerDataUpdate(const lpcs::Slice& data, const common::Point& axis_position) {
  if (auto* handler = GetActiveHandler()) {
    handler->OnScannerDataUpdate(data, axis_position);
  }
}

auto CalibrationManagerImpl::WeldObjectCalibrationValid() const -> bool { return cw_handler_->HasValidCalibration(); };

void CalibrationManagerImpl::Subscribe(std::function<void()> subscriber) {
  calibration_status_subscriber_ = subscriber;
};

auto CalibrationManagerImpl::GetActiveHandler() -> CalibrationHandler* {
  const auto status = handler_context_.activity_status->Get();
  if (cw_handler_->CanHandle(status)) {
    return cw_handler_.get();
  }
  if (lw_handler_->CanHandle(status)) {
    return lw_handler_.get();
  }
  return nullptr;
}
