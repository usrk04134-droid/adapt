#include "management_server.h"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

#include "common/logging/application_log.h"
#include "common/messages/management.h"
#include "common/zevs/zevs_socket.h"
#include "coordination/activity_status.h"
#include "coordination/calibration_status.h"
#include "joint_geometry/joint_geometry_provider.h"
#include "tracking/tracking_manager.h"
#include "weld_control/weld_control.h"
#include "weld_control/weld_control_types.h"

using management::ManagementServer;

ManagementServer::ManagementServer(zevs::Socket* socket, joint_geometry::JointGeometryProvider* joint_geometry_provider,
                                   coordination::ActivityStatus* activity_status,
                                   coordination::CalibrationStatus* calibration_status_v2,
                                   weld_control::WeldControl* weld_control, std::function<void()> shutdown_handler)
    : socket_(socket),
      joint_geometry_provider_(joint_geometry_provider),
      activity_status_(activity_status),
      calibration_status_v2_(calibration_status_v2),
      weld_control_(weld_control),
      shutdown_handler_(shutdown_handler) {
  LOG_DEBUG("Creating ManagementServer");
  weld_control_->SetObserver(this);

  auto on_weld_control_ready_update =
      [this](const std::vector<std::pair<weld_control::Mode, weld_control::LayerType>>& ready_modes) {
        weld_control_jt_ready_ =
            std::ranges::find(ready_modes, std::make_pair(weld_control::Mode::JOINT_TRACKING,
                                                          weld_control::LayerType::NOT_APPLICABLE)) !=
            ready_modes.end();
        weld_control_abp_ready_ =
            std::ranges::find(ready_modes, std::make_pair(weld_control::Mode::AUTOMATIC_BEAD_PLACEMENT,
                                                          weld_control::LayerType::FILL)) != ready_modes.end();

        weld_control_abp_cap_ready_ =
            std::ranges::find(ready_modes, std::make_pair(weld_control::Mode::AUTOMATIC_BEAD_PLACEMENT,
                                                          weld_control::LayerType::CAP)) != ready_modes.end();

        if (this->UpdateReadyState()) {
          this->SendReadyState();
        };
      };
  weld_control_->SubscribeReady(on_weld_control_ready_update);

  auto check_ready_state = [this]() {
    if (this->UpdateReadyState()) {
      this->SendReadyState();
    };
  };

  auto on_calibration_changed = [this, check_ready_state]() {
    weld_control_->ResetGrooveData();
    check_ready_state();
  };
  calibration_status_v2_->Subscribe(on_calibration_changed);
  activity_status_->Subscribe(check_ready_state);
  joint_geometry_provider_->Subscribe(check_ready_state);

  UpdateReadyState();

  socket_->Serve(&ManagementServer::OnSubscribeReadyState, this);
  socket_->Serve(&ManagementServer::OnTrackingStart, this);
  socket_->Serve(&ManagementServer::OnTrackingUpdate, this);
  socket_->Serve(&ManagementServer::OnStop, this);
  socket_->Serve(&ManagementServer::OnABPStart, this);
  socket_->Serve(&ManagementServer::OnABPStop, this);
  socket_->Serve(&ManagementServer::OnABPCapStart, this);
  socket_->Serve(&ManagementServer::OnABPCapStop, this);
  socket_->Serve(&ManagementServer::OnShutdown, this);
}

void ManagementServer::OnTrackingStart(common::msg::management::TrackingStart data) {
  auto joint_geometry = joint_geometry_provider_->GetJointGeometry();
  if (!joint_geometry.has_value()) {
    // This should not happen since check is part of ready_state
    LOG_ERROR("Cannot start Joint tracking, JointGeometry not available");
    return;
  }

  LOG_DEBUG("ManagementServer Starting joint tracking");

  activity_status_->Set(coordination::ActivityStatusE::TRACKING);
  auto tracking_mode = static_cast<tracking::TrackingMode>(data.joint_tracking_mode);

  weld_control_->JointTrackingStart(joint_geometry.value(), tracking_mode, data.horizontal_offset,
                                    data.vertical_offset);
}

void ManagementServer::OnSubscribeReadyState(common::msg::management::SubscribeReadyState /*data*/) {
  ready_state_subscribed_ = true;

  SendReadyState();
}

void ManagementServer::OnTrackingUpdate(common::msg::management::TrackingUpdate data) {
  LOG_DEBUG("ManagementServer::OnTrackingUpdate, joint_tracking_mode={}, horizontal_offset={}, vertical_offset={}",
            data.joint_tracking_mode, data.horizontal_offset, data.vertical_offset);

  auto tracking_mode = static_cast<tracking::TrackingMode>(data.joint_tracking_mode);
  weld_control_->JointTrackingUpdate(tracking_mode, data.horizontal_offset, data.vertical_offset);
}

void ManagementServer::OnStop(common::msg::management::Stop /*data*/) {
  if (activity_status_->Get() != coordination::ActivityStatusE::TRACKING) {
    LOG_INFO("ManagementServer::OnStop, incorrect status:{}", static_cast<uint32_t>(activity_status_->Get()));
    return;
  }

  StopActiveFunction();
}

void ManagementServer::OnABPStart(common::msg::management::ABPStart /*data*/) {
  if (activity_status_->Get() != coordination::ActivityStatusE::TRACKING) {
    LOG_ERROR("Cannot start ABP - status:{}", static_cast<uint32_t>(activity_status_->Get()));
    return;
  }
  LOG_DEBUG("ManagementServer Starting Automatic Bead Placement");
  weld_control_->AutoBeadPlacementStart(weld_control::LayerType::FILL);
}

void ManagementServer::OnABPStop(common::msg::management::ABPStop /*data*/) {
  LOG_DEBUG("ManagementServer Stopping Automatic Bead Placement");
  weld_control_->AutoBeadPlacementStop();
}

void ManagementServer::OnABPCapStart(common::msg::management::ABPCapStart /*data*/) {
  if (activity_status_->Get() != coordination::ActivityStatusE::TRACKING) {
    LOG_ERROR("Cannot start ABP CAP - status:{}", static_cast<uint32_t>(activity_status_->Get()));
    return;
  }
  LOG_DEBUG("ManagementServer Starting Automatic CAP Bead Placement");
  weld_control_->AutoBeadPlacementStart(weld_control::LayerType::CAP);
}

void ManagementServer::OnABPCapStop(common::msg::management::ABPCapStop /*data*/) {
  LOG_DEBUG("ManagementServer Stopping Automatic CAP Bead Placement");
  weld_control_->AutoBeadPlacementStop();
}

void ManagementServer::OnShutdown(common::msg::management::Shutdown /*data*/) {
  LOG_INFO("Shutdown received");
  shutdown_handler_();
}

void ManagementServer::StopActiveFunction() {
  if (activity_status_->Get() != coordination::ActivityStatusE::TRACKING) {
    return;
  }

  LOG_DEBUG("ManagementServer Stopping joint tracking / abp");
  weld_control_->Stop();

  activity_status_->Set(coordination::ActivityStatusE::IDLE);
}

auto ManagementServer::UpdateReadyState() -> bool {
  bool busy_from_webhmi{};

  switch (activity_status_->Get()) {
    case coordination::ActivityStatusE::IDLE:
    case coordination::ActivityStatusE::TRACKING:
      busy_from_webhmi = false;
      break;
    default:
      busy_from_webhmi = true;
      break;
  }
  auto weld_object_cal_valid = calibration_status_v2_->WeldObjectCalibrationValid();
  auto joint_geometry        = joint_geometry_provider_->GetJointGeometry();

  ReadyState new_ready_state = ReadyState::NOT_READY;
  if (activity_status_->Get() == coordination::ActivityStatusE::CALIBRATION_AUTO_MOVE) {
    new_ready_state = ReadyState::NOT_READY_AUTO_CAL_MOVE;
  } else {
    if (!busy_from_webhmi && weld_object_cal_valid && joint_geometry.has_value() && weld_control_jt_ready_) {
      new_ready_state = ReadyState::TRACKING_READY;
    }

    if (new_ready_state == ReadyState::TRACKING_READY) {
      if (weld_control_abp_ready_ && weld_control_abp_cap_ready_) {
        new_ready_state = ReadyState::ABP_AND_ABP_CAP_READY;
      } else if (weld_control_abp_ready_) {
        new_ready_state = ReadyState::ABP_READY;
      } else if (weld_control_abp_cap_ready_) {
        new_ready_state = ReadyState::ABP_CAP_READY;
      }
    }
  }

  if (new_ready_state != ready_state_) {
    LOG_INFO(
        "Changed ready state {} -> {}, woc: {}, jg: {}, busy_from_webhmi: {}, "
        "weld_control_jt_ready: {}, weld_control_abp_ready: {}, weld_control_abp_cap_ready: {}, activity_status: {}",
        ToString(ready_state_), ToString(new_ready_state), weld_object_cal_valid, joint_geometry.has_value(),
        busy_from_webhmi, weld_control_jt_ready_, weld_control_abp_ready_, weld_control_abp_cap_ready_,
        ActivityStatusToString(activity_status_->Get()));

    ready_state_ = new_ready_state;
    return true;
  }
  LOG_INFO(
      "Unchanged ready state: {}, woc: {}, jg: {}, busy_from_webhmi: {}, weld_control_jt_ready: {}, "
      "weld_control_abp_ready: {}, weld_control_abp_cap_ready: {}, activity_status: {}",
      ToString(ready_state_), weld_object_cal_valid, joint_geometry.has_value(), busy_from_webhmi,
      weld_control_jt_ready_, weld_control_abp_ready_, weld_control_abp_cap_ready_,
      ActivityStatusToString(activity_status_->Get()));

  return false;
}

void ManagementServer::SendReadyState() {
  if (!ready_state_subscribed_) {
    return;
  }

  common::msg::management::ReadyState data{};

  switch (ready_state_) {
    case ReadyState::NOT_READY:
      data.state = common::msg::management::ReadyState::State::NOT_READY;
      break;
    case ReadyState::NOT_READY_AUTO_CAL_MOVE:
      data.state = common::msg::management::ReadyState::State::NOT_READY_AUTO_CAL_MOVE;
      break;
    case ReadyState::TRACKING_READY:
      data.state = common::msg::management::ReadyState::State::TRACKING_READY;
      break;
    case ReadyState::ABP_READY:
      data.state = common::msg::management::ReadyState::State::ABP_READY;
      break;
    case ReadyState::ABP_CAP_READY:
      data.state = common::msg::management::ReadyState::State::ABP_CAP_READY;
      break;
    case ReadyState::ABP_AND_ABP_CAP_READY:
      data.state = common::msg::management::ReadyState::State::ABP_AND_ABP_CAP_READY;
      break;
  }

  socket_->Send(data);
}

void ManagementServer::OnNotifyHandoverToManual() {
  LOG_DEBUG("Notify Hanover to manual");
  socket_->Send(common::msg::management::NotifyHandoverToManual{});
}

void ManagementServer::OnGrooveDataTimeout() {
  LOG_DEBUG("Groove data timeout, stop joint tracking / abp");
  socket_->Send(common::msg::management::TrackingStoppedGrooveDataTimeout{});
  StopActiveFunction();
}

void ManagementServer::OnError() {
  LOG_DEBUG("OnError, stop joint tracking / abp");
  socket_->Send(common::msg::management::ScannerError{});
  StopActiveFunction();
}

void ManagementServer::OnGracefulStop() {
  LOG_DEBUG("Graceful Stop");
  socket_->Send(common::msg::management::GracefulStop{});
  StopActiveFunction();
}
