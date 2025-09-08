#include "kinematics_server_impl.h"

#include <chrono>
#include <cstdint>

#include "common/clock_functions.h"
#include "common/logging/application_log.h"
#include "common/messages/kinematics.h"
#include "common/zevs/zevs_socket.h"
#include "controller/controller_data.h"
#include "controller/kinematics_server.h"
#include "controller/slide_position_buffer.h"
#include "controller/weld_axis_buffer.h"

using controller::KinematicsServerImpl;
using std::chrono::milliseconds;

namespace controller {

const float SLIDE_ACCELERATION_HORIZONTAL = 30.0;
const float SLIDE_JERK_HORIZONTAL         = 100.0;

const float SLIDE_ACCELERATION_VERTICAL = 30.0;
const float SLIDE_JERK_VERTICAL         = 100.0;

enum class AxisId : uint32_t {
  INVALID,
  SLIDE_CROSS_HORIZONTAL,
  SLIDE_CROSS_VERTICAL,
  WELD_AXIS,
  SIZE,
};
}  // namespace controller

KinematicsServerImpl::KinematicsServerImpl(zevs::Socket* socket, KinematicsServerObserver* observer,
                                           SlidePositionBuffer* horizontal_position_buffer,
                                           SlidePositionBuffer* vertical_position_buffer,
                                           WeldAxisBuffer* weld_axis_buffer,
                                           clock_functions::SystemClockNowFunc system_clock_now_func)
    : socket_(socket),
      observer_(observer),
      horizontal_position_buffer_(horizontal_position_buffer),
      vertical_position_buffer_(vertical_position_buffer),
      weld_axis_buffer_(weld_axis_buffer),
      system_clock_now_func_(system_clock_now_func) {
  socket_->Serve(&KinematicsServerImpl::OnSetSlidesPosition, this);
  socket_->Serve(&KinematicsServerImpl::OnGetSlidesPosition, this);
  socket_->Serve(&KinematicsServerImpl::OnGetSlidesStatus, this);
  socket_->Serve(&KinematicsServerImpl::OnGetWeldAxisData, this);
  socket_->Serve(&KinematicsServerImpl::OnSetWeldAxisData, this);
  socket_->Serve(&KinematicsServerImpl::OnRelease, this);
  socket_->Serve(&KinematicsServerImpl::OnSubscribeStateChanges, this);
  socket_->Serve(&KinematicsServerImpl::OnUnSubscribeStateChanges, this);
  socket_->Serve(&KinematicsServerImpl::OnGetEdgePosition, this);
}

void KinematicsServerImpl::OnAxisInput(AxisInput axis) {
  uint64_t time_since_epoch = system_clock_now_func_().time_since_epoch().count();

  switch (static_cast<AxisId>(axis.get_axis_id())) {
    case AxisId::SLIDE_CROSS_HORIZONTAL:
      horizontal_in_position_ = axis.get_status_in_position();
      horizontal_position_buffer_->StorePosition(axis.get_position(), time_since_epoch);
      break;
    case AxisId::SLIDE_CROSS_VERTICAL:
      vertical_in_position_ = axis.get_status_in_position();
      vertical_position_buffer_->StorePosition(axis.get_position(), time_since_epoch);
      break;
    case AxisId::WELD_AXIS: {
      weld_axis_buffer_->StorePosition(axis.get_position(), axis.get_velocity(), time_since_epoch);

      auto const weld_axis_homed = axis.get_status_homed();
      if (weld_axis_homed != weld_axis_homed_) {
        weld_axis_homed_ = weld_axis_homed;

        if (send_state_changes_) {
          socket_->Send(common::msg::kinematics::StateChange{
              .weld_axis_state = weld_axis_homed_ ? common::msg::kinematics::StateChange::State::HOMED
                                                  : common::msg::kinematics::StateChange::State::INIT,
          });
        }
      }

    } break;
    default:
      LOG_ERROR("Invalid AxisID={}", axis.get_axis_id());
  }
}

void KinematicsServerImpl::OnWeldObjectRadius(double radius) { weld_object_radius_ = radius; }

void KinematicsServerImpl::OnEdgePositionAvailableStatus(bool status) {
  if (send_state_changes_ && (status != edge_position_available_)) {
    socket_->Send(common::msg::kinematics::EdgeStateChange{
        .edge_state = status ? common::msg::kinematics::EdgeStateChange::State::AVAILABLE
                             : common::msg::kinematics::EdgeStateChange::State::NOT_AVAILABLE});
  }
  edge_position_available_ = status;
}

void KinematicsServerImpl::OnEdgePosition(double value) { edge_position_ = value; }

void KinematicsServerImpl::OnSetSlidesPosition(common::msg::kinematics::SetSlidesPosition data) {
  LOG_TRACE("SetSlidesPosition - Horizontal:{:.2f}, Vertical:{:.2f}", data.horizontal, data.vertical);
  SetHorizontalAxis(data.horizontal, data.horizontal_lin_velocity);
  SetVerticalAxis(data.vertical, data.vertical_velocity);
}

void KinematicsServerImpl::OnGetSlidesPosition(common::msg::kinematics::GetSlidesPosition data) {
  uint64_t const time_stamp =
      data.time_stamp != 0 ? data.time_stamp : system_clock_now_func_().time_since_epoch().count();
  auto horizontal_position = horizontal_position_buffer_->GetPosition(time_stamp);
  auto vertical_position   = vertical_position_buffer_->GetPosition(time_stamp);

  socket_->Send(common::msg::kinematics::GetSlidesPositionRsp{.client_id  = data.client_id,
                                                              .time_stamp = data.time_stamp,
                                                              .horizontal = horizontal_position,
                                                              .vertical   = vertical_position});
}

void KinematicsServerImpl::OnGetSlidesStatus(common::msg::kinematics::GetSlidesStatus data) {
  socket_->Send(common::msg::kinematics::GetSlidesStatusRsp{.client_id              = data.client_id,
                                                            .horizontal_in_position = horizontal_in_position_,
                                                            .vertical_in_position   = vertical_in_position_});
}

void KinematicsServerImpl::OnRelease(common::msg::kinematics::Release /*data*/) {
  LOG_INFO("Release");
  observer_->Release();
}

void KinematicsServerImpl::SetHorizontalAxis(double position, double velocity) {
  AxisOutput data{};

  data.set_axis_id(static_cast<uint32_t>(AxisId::SLIDE_CROSS_HORIZONTAL));
  data.set_commands_execute(true);
  data.set_commands_stop(false);
  data.set_commands_follow_position(false);
  data.set_position(static_cast<float>(position));
  data.set_velocity(static_cast<float>(velocity));
  data.set_acceleration(SLIDE_ACCELERATION_HORIZONTAL);
  data.set_jerk(SLIDE_JERK_HORIZONTAL);

  observer_->OnAxisOutput(data);
}

void KinematicsServerImpl::SetVerticalAxis(double position, double vertical_velocity) {
  AxisOutput data{};

  data.set_axis_id(static_cast<uint32_t>(AxisId::SLIDE_CROSS_VERTICAL));
  data.set_commands_execute(true);
  data.set_commands_stop(false);
  data.set_commands_follow_position(false);
  data.set_position(static_cast<float>(position));
  data.set_velocity(static_cast<float>(vertical_velocity));
  data.set_acceleration(SLIDE_ACCELERATION_VERTICAL);
  data.set_jerk(SLIDE_JERK_VERTICAL);

  observer_->OnAxisOutput(data);
}

void KinematicsServerImpl::OnGetWeldAxisData(common::msg::kinematics::GetWeldAxisData data) {
  uint64_t const time_stamp =
      data.time_stamp != 0 ? data.time_stamp : system_clock_now_func_().time_since_epoch().count();
  auto axis_data = weld_axis_buffer_->GetPosition(time_stamp);

  common::msg::kinematics::GetWeldAxisDataRsp const rsp{
      .client_id  = data.client_id,
      .time_stamp = data.time_stamp,
      .position   = axis_data.position,
      .velocity   = axis_data.velocity,
      .radius     = weld_object_radius_,
  };
  socket_->Send(rsp);
}

void KinematicsServerImpl::OnSetWeldAxisData(common::msg::kinematics::SetWeldAxisData data) {
  LOG_TRACE("SetWeldAxisData - velocity:{:.2f}", data.velocity);
  AxisOutput axis_output_data{};
  axis_output_data.set_axis_id(static_cast<uint32_t>(AxisId::WELD_AXIS));
  axis_output_data.set_velocity(static_cast<float>(data.velocity));

  observer_->OnAxisOutput(axis_output_data);
}

void KinematicsServerImpl::OnSubscribeStateChanges(common::msg::kinematics::SubscribeStateChanges /*data*/) {
  send_state_changes_ = true;

  if (send_state_changes_) {
    /* send current states to new subscribers */
    socket_->Send(common::msg::kinematics::StateChange{
        .weld_axis_state = weld_axis_homed_ ? common::msg::kinematics::StateChange::State::HOMED
                                            : common::msg::kinematics::StateChange::State::INIT,
    });
    socket_->Send(common::msg::kinematics::EdgeStateChange{
        .edge_state = edge_position_available_ ? common::msg::kinematics::EdgeStateChange::State::AVAILABLE
                                               : common::msg::kinematics::EdgeStateChange::State::NOT_AVAILABLE});
  }
}

void KinematicsServerImpl::OnUnSubscribeStateChanges(common::msg::kinematics::UnSubscribeStateChanges /*data*/) {
  send_state_changes_ = false;
}

void KinematicsServerImpl::OnGetEdgePosition(common::msg::kinematics::GetEdgePosition data) {
  socket_->Send(common::msg::kinematics::GetEdgePositionRsp{.client_id = data.client_id, .position = edge_position_});
}
