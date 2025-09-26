#include "kinematics_client_impl.h"

#include <cstdint>
#include <utility>

#include "common/messages/kinematics.h"
#include "common/zevs/zevs_socket.h"
#include "kinematics/kinematics_client.h"

namespace kinematics {

KinematicsClientImpl::KinematicsClientImpl(zevs::Socket* socket) : socket_(socket) {
  socket_->Serve(&KinematicsClientImpl::OnGetSlidesPositionRsp, this);
  socket_->Serve(&KinematicsClientImpl::OnGetSlidesStatusRsp, this);
  socket_->Serve(&KinematicsClientImpl::OnGetWeldAxisDataRsp, this);
  socket_->Serve(&KinematicsClientImpl::OnKinematicsStateChange, this);
  socket_->Serve(&KinematicsClientImpl::OnEdgeStateChange, this);
  socket_->Serve(&KinematicsClientImpl::OnGetEdgePositionRsp, this);

  socket_->Send(common::msg::kinematics::SubscribeStateChanges{});
}

void KinematicsClientImpl::SetSlidesPosition(double horizontal, double vertical, double horizontal_lin_velocity,
                                             double vertical_velocity) {
  socket_->Send(common::msg::kinematics::SetSlidesPosition{.horizontal              = horizontal,
                                                           .vertical                = vertical,
                                                           .horizontal_lin_velocity = horizontal_lin_velocity,
                                                           .vertical_velocity       = vertical_velocity});
}

void KinematicsClientImpl::OnGetSlidesPositionRsp(common::msg::kinematics::GetSlidesPositionRsp data) {
  auto const transaction_id = data.client_id;
  auto iter                 = transactions_.find(transaction_id);
  if (iter != transactions_.end()) {
    auto const transaction = iter->second;
    transactions_.erase(iter);

    transaction.on_get_slides_position(data.time_stamp, data.horizontal, data.vertical);
  }
}

void KinematicsClientImpl::OnGetSlidesStatusRsp(common::msg::kinematics::GetSlidesStatusRsp data) {
  auto const transaction_id = data.client_id;
  auto iter                 = transactions_.find(transaction_id);
  if (iter != transactions_.end()) {
    auto const transaction = iter->second;
    transactions_.erase(iter);

    transaction.on_get_slides_status(data.horizontal_in_position, data.vertical_in_position);
  }
}

void KinematicsClientImpl::OnGetWeldAxisDataRsp(common::msg::kinematics::GetWeldAxisDataRsp data) {
  auto const transaction_id = data.client_id;
  auto iter                 = transactions_.find(transaction_id);
  if (iter != transactions_.end()) {
    auto const transaction = iter->second;
    transactions_.erase(iter);

    transaction.on_get_weld_axis_data(data.time_stamp, data.position, data.velocity, data.radius);
  }
}

void KinematicsClientImpl::GetSlidesPosition(OnGetSlidesPosition on_response) { GetSlidesPosition(0, on_response); }

void KinematicsClientImpl::GetSlidesPosition(std::uint64_t time_stamp, OnGetSlidesPosition on_response) {
  auto const transaction_id = ++transaction_id_;

  transactions_[transaction_id] = Transaction{
      .transaction_id         = transaction_id,
      .on_get_slides_position = on_response,
  };

  socket_->Send(common::msg::kinematics::GetSlidesPosition{.time_stamp = time_stamp, .client_id = transaction_id});
}

void KinematicsClientImpl::GetSlidesStatus(OnGetSlidesStatus on_response) {
  auto const transaction_id = ++transaction_id_;

  transactions_[transaction_id_] = Transaction{
      .transaction_id       = transaction_id,
      .on_get_slides_status = on_response,
  };

  socket_->Send(common::msg::kinematics::GetSlidesStatus{transaction_id});
}

void KinematicsClientImpl::GetWeldAxisData(std::uint64_t time_stamp, kinematics::OnGetWeldAxisData on_response) {
  auto const transaction_id = ++transaction_id_;

  transactions_[transaction_id] = Transaction{
      .transaction_id        = transaction_id,
      .on_get_weld_axis_data = on_response,
  };

  socket_->Send(common::msg::kinematics::GetWeldAxisData{
      .client_id  = transaction_id,
      .time_stamp = time_stamp,
  });
}

void KinematicsClientImpl::GetWeldAxisData(kinematics::OnGetWeldAxisData on_response) {
  GetWeldAxisData(0, on_response);
}

void KinematicsClientImpl::SetWeldAxisData(double velocity) {
  socket_->Send(common::msg::kinematics::SetWeldAxisData{.velocity = velocity});
}

void KinematicsClientImpl::Release() { socket_->Send(common::msg::kinematics::Release{}); }

void KinematicsClientImpl::OnKinematicsStateChange(common::msg::kinematics::StateChange data) {
  auto const state =
      data.weld_axis_state == common::msg::kinematics::StateChange::State::HOMED ? State::HOMED : State::INIT;
  auto const state_change = kinematics::StateChange{.weld_axis_state = state};
  for (auto const& [_, subscriber] : state_change_subscribers_) {
    subscriber.first(state_change);
  }
}

void KinematicsClientImpl::OnEdgeStateChange(common::msg::kinematics::EdgeStateChange data) {
  auto const edge_state = data.edge_state == common::msg::kinematics::EdgeStateChange::State::AVAILABLE
                              ? EdgeState::AVAILABLE
                              : EdgeState::NOT_AVAILABLE;
  for (auto const& [_, subscriber] : state_change_subscribers_) {
    subscriber.second(edge_state);
  }
}

auto KinematicsClientImpl::SubscribeStateChanges(OnStateChange on_state_change,
                                                 EdgeStateSubscriber on_edge_state_change) -> uint32_t {
  auto const handle                 = ++subscriber_index_;
  state_change_subscribers_[handle] = std::make_pair(on_state_change, on_edge_state_change);

  if (state_change_subscribers_.size() == 1) {
    socket_->Send(common::msg::kinematics::SubscribeStateChanges{});
  }

  return handle;
}

void KinematicsClientImpl::UnSubscribeStateChanges(uint32_t handle) {
  state_change_subscribers_.erase(handle);

  if (state_change_subscribers_.empty()) {
    socket_->Send(common::msg::kinematics::UnSubscribeStateChanges{});
  }
}

void KinematicsClientImpl::GetEdgePosition(OnGetEdgePosition on_response) {
  auto const transaction_id = ++transaction_id_;

  transactions_[transaction_id] = Transaction{
      .transaction_id       = transaction_id,
      .on_get_edge_position = on_response,
  };

  socket_->Send(common::msg::kinematics::GetEdgePosition{.client_id = transaction_id});
}

void KinematicsClientImpl::OnGetEdgePositionRsp(common::msg::kinematics::GetEdgePositionRsp data) {
  auto const transaction_id = data.client_id;
  auto iter                 = transactions_.find(transaction_id);
  if (iter != transactions_.end()) {
    auto const transaction = iter->second;
    transactions_.erase(iter);

    transaction.on_get_edge_position(data.position);
  }
}

}  // namespace kinematics
