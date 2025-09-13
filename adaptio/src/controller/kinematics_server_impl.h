#pragma once

#include "common/clock_functions.h"
#include "common/messages/kinematics.h"
#include "common/zevs/zevs_socket.h"
#include "controller/controller_data.h"
#include "kinematics_server.h"
#include "slide_position_buffer.h"
#include "weld_axis_buffer.h"

namespace controller {

class KinematicsServerImpl : public KinematicsServer {
 public:
  explicit KinematicsServerImpl(zevs::Socket* socket, KinematicsServerObserver* observer,
                                SlidePositionBuffer* horizontal_position_buffer,
                                SlidePositionBuffer* vertical_position_buffer, WeldAxisBuffer* weld_axis_buffer,
                                clock_functions::SystemClockNowFunc system_clock_now_func);

  void OnAxisInput(controller::AxisInput axis) override;
  void OnWeldObjectRadius(double radius) override;
  void OnEdgePositionAvailableStatus(bool status) override;
  void OnEdgePosition(double value) override;

 private:
  void OnGetSlidesPosition(common::msg::kinematics::GetSlidesPosition data);
  void OnSetSlidesPosition(common::msg::kinematics::SetSlidesPosition data);
  void OnGetSlidesStatus(common::msg::kinematics::GetSlidesStatus data);
  void OnRelease(common::msg::kinematics::Release data);
  void SetHorizontalAxis(double position, double horizontal_lin_velocity);
  void SetVerticalAxis(double position, double vertical_velocity);
  void OnGetWeldAxisData(common::msg::kinematics::GetWeldAxisData data);
  void OnSetWeldAxisData(common::msg::kinematics::SetWeldAxisData data);
  void OnSubscribeStateChanges(common::msg::kinematics::SubscribeStateChanges data);
  void OnUnSubscribeStateChanges(common::msg::kinematics::UnSubscribeStateChanges data);
  void OnGetEdgePosition(common::msg::kinematics::GetEdgePosition data);

  // TriState enum type only because optional<bool> is very error prone
  enum class TriState {
    UNKNOWN,
    TRUE,
    FALSE,
  };

  zevs::Socket* socket_;
  KinematicsServerObserver* observer_ = nullptr;
  SlidePositionBuffer* horizontal_position_buffer_;
  SlidePositionBuffer* vertical_position_buffer_;
  WeldAxisBuffer* weld_axis_buffer_;
  bool horizontal_in_position_{false};
  bool vertical_in_position_{false};
  double weld_object_radius_{0.};
  clock_functions::SystemClockNowFunc system_clock_now_func_;
  TriState weld_axis_homed_{TriState::UNKNOWN};
  bool send_state_changes_ = false;
  TriState edge_position_available_{TriState::UNKNOWN};
  double edge_position_{};
};

}  // namespace controller
