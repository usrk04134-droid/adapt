#include "scanner/scanner_application.h"

#include <fmt/core.h>
#include <prometheus/registry.h>

#include <Eigen/Eigen>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <tuple>

#include "common/logging/application_log.h"
#include "common/messages/scanner.h"
#include "common/zevs/zevs_core.h"
#include "common/zevs/zevs_socket.h"
#include "scanner/core/scanner_calibration_configuration.h"
#include "scanner/core/scanner_configuration.h"
#include "scanner/core/scanner_factory.h"
#include "scanner/core/scanner_types.h"
#include "scanner/core/src/scanner_impl.h"
#include "scanner/core/src/scanner_server.h"
#include "scanner/image/image.h"
#include "scanner/image_logger/image_logger.h"
#include "scanner/image_provider/image_provider.h"
#include "scanner/image_provider/image_provider_configuration.h"
#include "scanner/joint_model/joint_model.h"

namespace scanner {

const double METER_PER_MM = 1.e-3;

ScannerApplication::ScannerApplication(const ScannerConfigurationData& scanner_config,
                                       const ScannerCalibrationData& scanner_calib, const image_provider::Fov& fov,
                                       image_provider::ImageProvider* image_provider,
                                       const std::string& endpoint_base_url,
                                       const std::optional<std::filesystem::path>& data_path,
                                       image_logger::ImageLogger* image_logger, prometheus::Registry* registry)
    : scanner_config_(scanner_config),
      scanner_calib_(scanner_calib),
      image_provider_(image_provider),
      image_logger_(image_logger),
      data_path_(data_path),
      fov_(fov),
      registry_(registry) {
  LOG_DEBUG("Creating ScannerApplication");
  scanner_endpoint_url_ = fmt::format("inproc://{}/scanner", endpoint_base_url);
}

void ScannerApplication::Start(common::msg::scanner::Start data) {
  LOG_DEBUG("Start");

  if (timer_task_id_.has_value()) {
    timer_->Cancel(timer_task_id_.value());
    timer_task_id_ = {};
    LOG_DEBUG("ScannerApplication::Stop, timer cancelled");
  }

  auto const properties = MsgDataToJointProperties(data.joint_geometry);
  image_logger_->AddMetaData(joint_model::ToYaml(properties), ToYaml(scanner_calib_));

  if (core_scanner_) {
    core_scanner_->Stop();
  }

  core_scanner_ = GetCoreScanner(properties);
  if (!core_scanner_) {
    LOG_ERROR("Scanner setup failed!");
    socket_->Send(common::msg::scanner::StartRsp{.success = false});
    return;
  }

  if (data.interval.count() == 0) {
    LOG_ERROR("Invalid scanner input interval");
    socket_->Send(common::msg::scanner::StartRsp{.success = false});
    return;
  }

  auto sensitivity = ScannerSensitivity::NORMAL;
  switch (data.sensitivity) {
    case common::msg::scanner::ScannerSensitivity::NORMAL:
      break;
    case common::msg::scanner::ScannerSensitivity::HIGH:
      sensitivity = ScannerSensitivity::HIGH;
      break;
  };

  auto result = core_scanner_->Start(sensitivity);
  if (result == boost::outcome_v2::success()) {
    timer_task_id_ =
        timer_->RequestPeriodic(&ScannerApplication::OnTimeout, this, data.interval.count(), "poll_image_periodic");
    LOG_DEBUG("ScannerApplication::Start, Timer period (ms)={}", data.interval.count());
    socket_->Send(common::msg::scanner::StartRsp{.success = true});
  } else {
    LOG_ERROR("Not able to start scanner");
    socket_->Send(common::msg::scanner::StartRsp{.success = false});
  }
}

void ScannerApplication::Stop(common::msg::scanner::Stop /*unused*/) {
  LOG_DEBUG("Stop");

  if (timer_task_id_.has_value()) {
    timer_->Cancel(timer_task_id_.value());
    timer_task_id_ = {};
    LOG_DEBUG("ScannerApplication::Stop, timer cancelled");
  }

  if (!core_scanner_) {
    LOG_ERROR("No setup of scanner is done");
    socket_->Send(common::msg::scanner::StopRsp{false});
    return;
  }

  core_scanner_->Stop();
  socket_->Send(common::msg::scanner::StopRsp{true});
}

void ScannerApplication::Update(common::msg::scanner::Update data) {
  LOG_DEBUG("Update");
  auto abw0_abw6_horizontla = std::make_tuple(METER_PER_MM * data.abw0_horizontal, METER_PER_MM * data.abw6_horizontal);
  core_scanner_->UpdateJointApproximation(MsgDataToJointProperties(data.joint_geometry), abw0_abw6_horizontla);
}

void ScannerApplication::ImageLoggingUpdate(common::msg::scanner::ImageLoggingData data) {
  LOG_DEBUG("ImageLoggingUpdate");

  image_logger::ImageLoggerConfig config = {
      .path              = std::filesystem::path(data.path),
      .sample_rate       = data.sample_rate,
      .buffer_size       = data.depth,
      .on_error_interval = data.on_error_interval,
  };

  auto path = std::string(data.path);
  if (path.length() > 0) {
    config.path = path;
  }

  switch (data.mode) {
    case common::msg::scanner::ImageLoggerMode::OFF:
      config.mode = image_logger::ImageLoggerConfig::Mode::OFF;
      break;
    case common::msg::scanner::ImageLoggerMode::DIRECT:
      config.mode = image_logger::ImageLoggerConfig::Mode::DIRECT;
      break;
    case common::msg::scanner::ImageLoggerMode::BUFFERED:
      config.mode = image_logger::ImageLoggerConfig::Mode::BUFFERED;
      break;
    case common::msg::scanner::ImageLoggerMode::ON_ERROR:
      config.mode = image_logger::ImageLoggerConfig::Mode::ON_ERROR;
      break;
  }

  image_logger_->Update(config);
}

void ScannerApplication::FlushImageBuffer(common::msg::scanner::FlushImageBuffer /*unused*/) {
  LOG_DEBUG("FlushImageBuffer");

  image_logger_->FlushBuffer();
}

void ScannerApplication::OnTimeout() { core_scanner_->Update(); }

auto ScannerApplication::GetCoreScanner(const joint_model::JointProperties& properties) -> ScannerPtr {
  LOG_DEBUG("GetCoreScanner");
  scanner_server_ = std::make_unique<ScannerServer>(socket_);

  auto scanner = scanner::GetFactory()->CreateScanner(image_provider_, scanner_calib_, scanner_config_, fov_,
                                                      properties, scanner_server_.get(), image_logger_, registry_);

  if (test_post_exec_) {
    if (auto* impl = dynamic_cast<ScannerImpl*>(scanner.get())) {
      impl->SetPostExecutorForTests(test_post_exec_);
    }
  }
  return scanner;
}

void ScannerApplication::StartThread(const std::string& event_loop_name) {
  thread_ = std::thread(&ScannerApplication::ThreadEntry, this, event_loop_name);
}

// Exit eventloop is done in exithandler in main.cc
void ScannerApplication::JoinThread() { thread_.join(); }

void ScannerApplication::ThreadEntry(const std::string& name) {
  LOG_DEBUG("Starting Scanner Messenger");
  event_loop_ = zevs::GetCoreFactory()->CreateEventLoop(name);

  socket_ = zevs::GetFactory()->CreatePairSocket(*event_loop_);
  socket_->Bind(scanner_endpoint_url_);

  socket_->Serve(&ScannerApplication::Start, this);
  socket_->Serve(&ScannerApplication::Stop, this);
  socket_->Serve(&ScannerApplication::Update, this);
  socket_->Serve(&ScannerApplication::ImageLoggingUpdate, this);
  socket_->Serve(&ScannerApplication::FlushImageBuffer, this);

  timer_ = zevs::GetFactory()->CreateTimer(*event_loop_);

  event_loop_->Run();
}

auto ScannerApplication::MsgDataToJointProperties(common::msg::scanner::JointGeometry joint_geometry)
    -> joint_model::JointProperties {
  joint_model::JointProperties properties = {.upper_joint_width           = joint_geometry.upper_joint_width_mm,
                                             .left_max_surface_angle      = joint_geometry.left_max_surface_angle_rad,
                                             .right_max_surface_angle     = joint_geometry.right_max_surface_angle_rad,
                                             .left_joint_angle            = joint_geometry.left_joint_angle_rad,
                                             .right_joint_angle           = joint_geometry.right_joint_angle_rad,
                                             .groove_depth                = joint_geometry.groove_depth_mm,
                                             .upper_joint_width_tolerance = joint_geometry.tolerance.upper_width_mm,
                                             .surface_angle_tolerance     = joint_geometry.tolerance.surface_angle_rad,
                                             .groove_angle_tolerance      = joint_geometry.tolerance.wall_angle_rad,
                                             .offset_distance             = 3.0};

  return properties;
}

void ScannerApplication::Exit() { event_loop_->Exit(); }

void ScannerApplication::SetTestPostExecutor(std::function<void(std::function<void()>)> exec) {
  test_post_exec_ = std::move(exec);
}
}  // namespace scanner
