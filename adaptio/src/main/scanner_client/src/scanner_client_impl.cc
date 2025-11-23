#include "scanner_client/src/scanner_client_impl.h"

#include <cmath>
#include <cstdint>
#include <vector>

#include "common/groove/point.h"
#include "common/logging/application_log.h"
#include "common/messages/scanner.h"
#include "common/zevs/zevs_socket.h"
#include "joint_geometry/joint_geometry.h"
#include "kinematics/kinematics_client.h"
#include "lpcs/lpcs_point.h"
#include "lpcs/lpcs_slice.h"
#include "scanner_client/scanner_client.h"

namespace scanner_client {

ScannerClientImpl::ScannerClientImpl(zevs::Socket* socket, kinematics::KinematicsClient* kinematics_client,
                                     double joint_tolerance_upper_width, double joint_tolerance_surface_angle,
                                     double joint_tolerance_wall_angle)
    : socket_(socket),
      kinematics_client_(kinematics_client),
      started_(false),
      joint_tolerance_upper_width_(joint_tolerance_upper_width),
      joint_tolerance_surface_angle_(joint_tolerance_surface_angle),
      joint_tolerance_wall_angle_(joint_tolerance_wall_angle) {
  LOG_DEBUG("Creating ScannerClientImpl");

  socket_->Serve(&ScannerClientImpl::OnSliceData, this);
  socket_->Serve(&ScannerClientImpl::OnStartRsp, this);
  socket_->Serve(&ScannerClientImpl::OnStopRsp, this);
}

void ScannerClientImpl::Start(const Config& config, const joint_geometry::JointGeometry& joint_geometry) {
  joint_geometry_ = {
      .upper_joint_width_mm        = joint_geometry.upper_joint_width_mm,
      .groove_depth_mm             = joint_geometry.groove_depth_mm,
      .left_joint_angle_rad        = joint_geometry.left_joint_angle_rad,
      .right_joint_angle_rad       = joint_geometry.right_joint_angle_rad,
      .left_max_surface_angle_rad  = joint_geometry.left_max_surface_angle_rad,
      .right_max_surface_angle_rad = joint_geometry.right_max_surface_angle_rad,
      .tolerance                   = {.upper_width_mm    = joint_tolerance_upper_width_,
                                      .surface_angle_rad = joint_tolerance_surface_angle_,
                                      .wall_angle_rad    = joint_tolerance_wall_angle_}
  };

  auto msg = common::msg::scanner::Start{
      .joint_geometry = joint_geometry_,
      .interval       = config.interval,
  };

  switch (config.sensitivity) {
    case ScannerSensitivity::NORMAL:
      msg.sensitivity = common::msg::scanner::ScannerSensitivity::NORMAL;
      break;
    case ScannerSensitivity::HIGH:
      msg.sensitivity = common::msg::scanner::ScannerSensitivity::HIGH;
      break;
  };

  started_ = true;

  socket_->Send(msg);
}

void ScannerClientImpl::Update(const UpdateData& data) {
  joint_geometry_.upper_joint_width_mm     = data.upper_width;
  joint_geometry_.left_joint_angle_rad     = data.left_wall_angle;
  joint_geometry_.right_joint_angle_rad    = data.right_wall_angle;
  joint_geometry_.tolerance.upper_width_mm = data.tolerance.upper_width,
  joint_geometry_.tolerance.wall_angle_rad = data.tolerance.wall_angle,

  socket_->Send(common::msg::scanner::Update{
      .joint_geometry  = joint_geometry_,
      .abw0_horizontal = data.abw0_horizontal,
      .abw6_horizontal = data.abw6_horizontal,
  });
}

void ScannerClientImpl::Stop() {
  LOG_DEBUG("Stop");
  started_ = false;
  socket_->Send(common::msg::scanner::Stop{});
}

void ScannerClientImpl::ImageLoggingUpdate(const ImageLoggingData& data) {
  common::msg::scanner::ImageLoggingData config = {
      .sample_rate       = data.sample_rate,
      .depth             = data.buffer_size,
      .on_error_interval = data.on_error_interval,
  };

  switch (data.mode) {
    case ImageLoggingData::Mode::OFF:
      config.mode = common::msg::scanner::ImageLoggerMode::OFF;
      break;
    case ImageLoggingData::Mode::DIRECT:
      config.mode = common::msg::scanner::ImageLoggerMode::DIRECT;
      break;
    case ImageLoggingData::Mode::BUFFERED:
      config.mode = common::msg::scanner::ImageLoggerMode::BUFFERED;
      break;
    case ImageLoggingData::Mode::ON_ERROR:
      config.mode = common::msg::scanner::ImageLoggerMode::ON_ERROR;
      break;
  }

  config.path[0]  = 0;
  auto const path = data.path.string();
  path.copy(config.path, data.path.string().length() + 1);

  socket_->Send(config);
}

void ScannerClientImpl::FlushImageBuffer() { socket_->Send(common::msg::scanner::FlushImageBuffer{}); }

void ScannerClientImpl::AddObserver(ScannerObserver* observer) { observers_.push_back(observer); }

void ScannerClientImpl::OnGetSlidesPosition(std::uint64_t time_stamp, double horizontal, double vertical) {
  LOG_TRACE("Scannerdata: {}, slides position: {:.2f}, {:.2f}", scanner_data_in_process_.Describe(), horizontal,
            vertical);
  common::Point position{.horizontal = horizontal, .vertical = vertical};

  if (scanner_data_in_process_.time_stamp == time_stamp) {
    for (auto& observer : observers_) {
      observer->OnScannerDataUpdate(scanner_data_in_process_, position);
    }
  } else {
    LOG_TRACE("Time stamp mismatch");
  }
  scanner_data_in_process_ = {};
}

void ScannerClientImpl::OnSliceData(common::msg::scanner::SliceData slice_data) {
  LOG_TRACE("ScannerClientImpl received slice data, request slides position");

  if (started_) {
    scanner_data_in_process_.groove = std::vector<lpcs::Point>{};
    for (auto& abws : slice_data.groove) {
      scanner_data_in_process_.groove->push_back({abws.x, abws.y});
    }

    for (auto& line : slice_data.profile) {
      scanner_data_in_process_.profile.push_back({line.horizontal, line.vertical});
    }

    scanner_data_in_process_.time_stamp  = slice_data.time_stamp;
    scanner_data_in_process_.groove_area = slice_data.groove_area;

    switch (slice_data.confidence) {
      case common::msg::scanner::SliceConfidence::NO:
        scanner_data_in_process_.confidence = lpcs::SliceConfidence::NO;
        break;
      case common::msg::scanner::SliceConfidence::LOW:
        scanner_data_in_process_.confidence = lpcs::SliceConfidence::LOW;
        break;
      case common::msg::scanner::SliceConfidence::MEDIUM:
        scanner_data_in_process_.confidence = lpcs::SliceConfidence::MEDIUM;
        break;
      case common::msg::scanner::SliceConfidence::HIGH:
        scanner_data_in_process_.confidence = lpcs::SliceConfidence::HIGH;
        break;
    }

    auto on_response = [this](std::uint64_t time_stamp, double horizontal, double vertical) {
      this->OnGetSlidesPosition(time_stamp, horizontal, vertical);
    };

    kinematics_client_->GetSlidesPosition(slice_data.time_stamp, on_response);
  }
}

void ScannerClientImpl::OnStartRsp(common::msg::scanner::StartRsp started) {
  LOG_DEBUG("OnScannerStarted");

  if (!started.success) {
    // Todo: what to do when not started?
    LOG_ERROR("Failed to start scanner");
  }
  for (auto& observer : observers_) {
    observer->OnScannerStarted(started.success);
  }
}

void ScannerClientImpl::OnStopRsp(common::msg::scanner::StopRsp stopped) {
  LOG_DEBUG("OnScannerStopped");

  if (!stopped.success) {
    // Todo: what to do when not started?
    LOG_ERROR("Failed to stop scanner");
  }
  for (auto& observer : observers_) {
    observer->OnScannerStopped(stopped.success);
  }
}

}  // namespace scanner_client
