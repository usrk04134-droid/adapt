#include "application.h"

#include <fmt/core.h>
#include <prometheus/registry.h>
#include <SQLiteCpp/Database.h>

#include <filesystem>
#include <memory>
#include <string>

#include "bead_control/src/bead_control_impl.h"
#include "bead_control/src/weld_position_data_storage.h"
#include "calibration/calibration_configuration.h"
#include "calibration/calibration_metrics.h"
#include "calibration/src/calibration_manager_v2_impl.h"
#include "calibration/src/calibration_solver_impl.h"
#include "cli_handler/log_level_cli.h"
#include "common/clock_functions.h"
#include "common/containers/relative_position_buffer.h"
#include "common/logging/application_log.h"
#include "common/zevs/zevs_core.h"
#include "common/zevs/zevs_socket.h"
#include "configuration/config_manager.h"
#include "coordination/activity_status_impl.h"
#include "event_handler/src/event_handler_impl.h"
#include "image_logging/src/image_logging_manager_impl.h"
#include "joint_geometry/src/joint_geometry_provider_impl.h"
#include "kinematics/kinematics_client_impl.h"
#include "management/management_server.h"
#include "scanner_client/scanner_client_impl.h"
#include "slice_translator/coordinates_translator.h"
#include "slice_translator/src/model_impl.h"
#include "tracking/src/tracking_manager_impl.h"
#include "web_hmi/src/service_mode_manager_impl.h"
#include "web_hmi/src/web_hmi_server.h"
#include "weld_control/src/delay_buffer.h"
#include "weld_control/src/settings_provider.h"
#include "weld_control/src/weld_control_impl.h"
#include "weld_control/src/weld_sequence_config_impl.h"
#include "weld_system_client/src/weld_system_client_impl.h"

Application::Application(configuration::ConfigManager* configuration, std::filesystem::path const& path_events,
                         SQLite::Database* database, std::filesystem::path const& path_logs,
                         clock_functions::SystemClockNowFunc system_clock_now_func,
                         clock_functions::SteadyClockNowFunc steady_clock_now_func, prometheus::Registry* registry,
                         int log_level)
    : database_(database),
      configuration_(configuration),
      path_logs_(path_logs),
      path_events_(path_events),
      system_clock_now_func_(system_clock_now_func),
      steady_clock_now_func_(steady_clock_now_func),
      registry_(registry),
      log_level_(log_level) {}

auto Application::Run(const std::string& event_loop_name, const std::string& endpoint_base_url) -> bool {
  LOG_TRACE("Starting Application");
  event_loop_name_ = event_loop_name;

  // Tolerances
  auto tolerances_configuration = configuration_->GetTolerancesConfiguration();

  event_loop_ = zevs::GetCoreFactory()->CreateEventLoop(event_loop_name_);
  timer_      = zevs::GetFactory()->CreateTimer(*event_loop_);

  event_handler_ = std::make_unique<event::EventHandlerImpl>(system_clock_now_func_, path_logs_, registry_);
  event_handler_->LoadEventsFromFile(path_events_);

  // KinematicsClient
  kinematics_socket_ = zevs::GetFactory()->CreatePairSocket(*event_loop_);
  kinematics_socket_->Connect(fmt::format("inproc://{}/kinematics", endpoint_base_url));
  kinematics_client_ = std::make_unique<kinematics::KinematicsClientImpl>(kinematics_socket_.get());

  // Scanner client
  scanner_client_socket_ = zevs::GetFactory()->CreatePairSocket(*event_loop_);
  scanner_client_socket_->Connect(fmt::format("inproc://{}/scanner", endpoint_base_url));
  scanner_client_ = std::make_unique<scanner_client::ScannerClientImpl>(
      scanner_client_socket_.get(), kinematics_client_.get(), tolerances_configuration.joint_geometry.upper_width,
      tolerances_configuration.joint_geometry.surface_angle, tolerances_configuration.joint_geometry.wall_angle);

  // WeldSystem
  weld_system_client_socket_ = zevs::GetFactory()->CreatePairSocket(*event_loop_);
  weld_system_client_socket_->Connect(fmt::format("inproc://{}/weld-system", endpoint_base_url));
  weld_system_client_ = std::make_unique<weld_system::WeldSystemClientImpl>(weld_system_client_socket_.get());

  // SliceTranslator
  model_impl_ = std::make_unique<slice_translator::ModelImpl>();

  // CoordinationStatus
  activity_status_ = std::make_unique<coordination::ActivityStatusImpl>();

  // JointTrackingManager
  tracking_manager_ = std::make_unique<tracking::TrackingManagerImpl>();

  // CoordinatesTranslator
  coordinates_translator_ = std::make_unique<slice_translator::CoordinatesTranslator>(model_impl_.get());
  scanner_client_->AddObserver(coordinates_translator_.get());

  // Management socket
  management_socket_ = zevs::GetFactory()->CreatePairSocket(*event_loop_);
  management_socket_->Connect(fmt::format("inproc://{}/management", endpoint_base_url));

  // WebHmi sockets
  web_hmi_in_socket_ =
      zevs::GetCoreFactory()->CreateCoreSocket(*event_loop_, zevs::SocketType::SUB, zevs::MessageType::RAW);
  web_hmi_in_socket_->SetFilter("");
  web_hmi_in_socket_->Bind(web_hmi_in_endpoint_url_);

  web_hmi_out_socket_ = zevs::GetCoreFactory()->CreateCoreSocket(zevs::SocketType::PUB, zevs::MessageType::RAW);
  web_hmi_out_socket_->Bind(web_hmi_out_endpoint_url_);

  joint_geometry_provider_ = std::make_unique<joint_geometry::JointGeometryProviderImpl>(configuration_, database_);

  // CalibrationMetrics
  calibration_metrics_ = std::make_unique<calibration::CalibrationMetrics>(registry_);

  // Service Mode
  web_hmi_server_ = std::make_unique<web_hmi::WebHmiServer>(web_hmi_in_socket_.get(), web_hmi_out_socket_.get(),
                                                            joint_geometry_provider_.get(), kinematics_client_.get(),
                                                            activity_status_.get(), calibration_metrics_.get());

  // Image logging manager
  auto image_logger_config = configuration_->GetImageLoggingConfiguration();
  if (image_logger_config.path.empty()) {
    image_logger_config.path = path_logs_ / "images";
  }
  image_logging_manager_ = std::make_unique<image_logging::ImageLoggingManagerImpl>(
      image_logger_config, web_hmi_server_.get(), scanner_client_.get());

  // Calibration V2
  calibration_solver_ = std::make_unique<calibration::CalibrationSolverImpl>(model_impl_.get());

  calibration_manager_v2_ = std::make_unique<calibration::CalibrationManagerV2Impl>(
      database_, timer_.get(), scanner_client_.get(), calibration_solver_.get(), model_impl_.get(),
      activity_status_.get(), web_hmi_server_.get(), joint_geometry_provider_.get(), system_clock_now_func_,
      kinematics_client_.get(), registry_, path_logs_, configuration_->GetCalibrationConfiguration().grid_config,
      configuration_->GetCalibrationConfiguration().runner_config);

  auto loglevel_cli = std::make_unique<cli_handler::LogLevelCli>(web_hmi_server_.get(), log_level_);

  event_handler_->SetWebHmi(web_hmi_server_.get());
  joint_geometry_provider_->SetWebHmi(web_hmi_server_.get());

  coordinates_translator_->AddObserver(web_hmi_server_.get());

  weld_pos_data_storage_ = std::make_unique<common::containers::RelativePositionBuffer<bead_control::WeldPositionData>>(
      bead_control::MAX_BUFFER_SIZE);
  bead_control_ = std::make_unique<bead_control::BeadControlImpl>(weld_pos_data_storage_.get(), steady_clock_now_func_);

  delay_buffer_ = std::make_unique<weld_control::DelayBuffer>(weld_control::DELAY_BUFFER_SIZE);

  weld_sequence_config_ = std::make_unique<weld_control::WeldSequenceConfigImpl>(database_, web_hmi_server_.get());
  settings_provider_    = std::make_unique<weld_control::SettingsProvider>(database_, web_hmi_server_.get());
  weld_control_         = std::make_unique<weld_control::WeldControlImpl>(
      configuration_->GetWeldControlConfiguration(), weld_sequence_config_.get(), settings_provider_.get(),
      web_hmi_server_.get(), kinematics_client_.get(), path_logs_, weld_system_client_.get(), tracking_manager_.get(),
      scanner_client_.get(), timer_.get(), event_handler_.get(), bead_control_.get(), delay_buffer_.get(),
      system_clock_now_func_, steady_clock_now_func_, registry_, image_logging_manager_.get(), model_impl_.get(),
      database_);
  service_mode_manager_ = std::make_unique<web_hmi::ServiceModeManagerImpl>(
      web_hmi_out_socket_.get(), kinematics_client_.get(), joint_geometry_provider_.get(), weld_control_.get(),
      activity_status_.get(), web_hmi_server_.get());

  auto shutdown_handler = [this]() {
    in_shutdown_ = true;
    this->Exit();
  };
  management_server_ = std::make_unique<management::ManagementServer>(
      management_socket_.get(), joint_geometry_provider_.get(), activity_status_.get(), calibration_manager_v2_.get(),
      weld_control_.get(), shutdown_handler);

  coordinates_translator_->AddObserver(weld_control_.get());
  scanner_client_->AddObserver(weld_control_.get());

  event_loop_->Run();
  return in_shutdown_;
}

auto Application::InShutdown() const -> bool { return in_shutdown_; }

void Application::Exit() { event_loop_->Exit(); }
