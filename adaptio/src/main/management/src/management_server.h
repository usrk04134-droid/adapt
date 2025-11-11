#pragma once

#include "common/messages/management.h"
#include "common/zevs/zevs_socket.h"
#include "coordination/activity_status.h"
#include "coordination/calibration_status.h"
#include "joint_geometry/joint_geometry_provider.h"
#include "weld_control/weld_control.h"

namespace management {

enum class ReadyState {
  NOT_READY,
  NOT_READY_AUTO_CAL_MOVE,
  TRACKING_READY,
  ABP_READY,
  ABP_CAP_READY,
  ABP_AND_ABP_CAP_READY,
};

inline auto ToString(ReadyState state) -> std::string {
  switch (state) {
    case ReadyState::NOT_READY:
      return "not_ready";
    case ReadyState::NOT_READY_AUTO_CAL_MOVE:
      return "not_ready_auto_cal_move";
    case ReadyState::TRACKING_READY:
      return "tracking_ready";
    case ReadyState::ABP_READY:
      return "abp_ready";
    case ReadyState::ABP_CAP_READY:
      return "abp_cap_ready";
    case ReadyState::ABP_AND_ABP_CAP_READY:
      return "abp_and_abp_cap_ready";
    default:
      return "";
  }
}

class ManagementServer : public weld_control::WeldControlObserver {
 public:
  ManagementServer(zevs::Socket* socket, joint_geometry::JointGeometryProvider* joint_geometry_provider,
                   coordination::ActivityStatus* activity_status, coordination::CalibrationStatus* calibration_status,
                   weld_control::WeldControl* weld_control, std::function<void()> shutdown_handler);

  // WeldControlObserver:
  void OnNotifyHandoverToManual() override;
  void OnGrooveDataTimeout() override;
  void OnError() override;
  void OnGracefulStop() override;

 private:
  void StopActiveFunction();
  auto UpdateReadyState() -> bool;
  void SendReadyState();

  // message handlers:
  void OnSubscribeReadyState(common::msg::management::SubscribeReadyState data);
  void OnTrackingStart(common::msg::management::TrackingStart data);
  void OnTrackingUpdate(common::msg::management::TrackingUpdate data);
  void OnStop(common::msg::management::Stop data);
  void OnABPStart(common::msg::management::ABPStart data);
  void OnABPStop(common::msg::management::ABPStop data);
  void OnABPCapStart(common::msg::management::ABPCapStart data);
  void OnABPCapStop(common::msg::management::ABPCapStop data);
  void OnShutdown(common::msg::management::Shutdown data);

  zevs::Socket* socket_;
  joint_geometry::JointGeometryProvider* joint_geometry_provider_;
  coordination::ActivityStatus* activity_status_;
  coordination::CalibrationStatus* calibration_status_;
  weld_control::WeldControl* weld_control_;
  std::function<void()> shutdown_handler_;
  bool ready_state_subscribed_ = false;
  ReadyState ready_state_;
  bool weld_control_jt_ready_      = false;
  bool weld_control_abp_ready_     = false;
  bool weld_control_abp_cap_ready_ = false;
};

}  // namespace management
