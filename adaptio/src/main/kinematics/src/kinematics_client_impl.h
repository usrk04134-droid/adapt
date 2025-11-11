#pragma once

#include <cstdint>
#include <map>
#include <utility>

#include "common/messages/kinematics.h"
#include "common/zevs/zevs_socket.h"
#include "kinematics/kinematics_client.h"

namespace kinematics {

class KinematicsClientImpl : public KinematicsClient {
 public:
  explicit KinematicsClientImpl(zevs::Socket* socket);

  void SetSlidesPosition(double horizontal, double vertical, double horizontal_lin_velocity,
                         double vertical_velocity) override;
  void GetSlidesPosition(OnGetSlidesPosition on_response) override;
  void GetSlidesPosition(std::uint64_t time_stamp, OnGetSlidesPosition on_response) override;
  void GetSlidesStatus(OnGetSlidesStatus on_response) override;
  void GetWeldAxisData(OnGetWeldAxisData on_response) override;
  void GetWeldAxisData(std::uint64_t time_stamp, OnGetWeldAxisData on_response) override;
  void SetWeldAxisData(double velocity) override;
  void Release() override;
  auto SubscribeStateChanges(OnStateChange on_state_change, EdgeStateSubscriber edge_state_subscriber)
      -> uint32_t override;
  void UnSubscribeStateChanges(uint32_t handle) override;
  void GetEdgePosition(OnGetEdgePosition on_response) override;

 private:
  void OnGetSlidesPositionRsp(common::msg::kinematics::GetSlidesPositionRsp data);
  void OnGetSlidesStatusRsp(common::msg::kinematics::GetSlidesStatusRsp data);
  void OnGetWeldAxisDataRsp(common::msg::kinematics::GetWeldAxisDataRsp data);
  void OnKinematicsStateChange(common::msg::kinematics::StateChange data);
  void OnEdgeStateChange(common::msg::kinematics::EdgeStateChange data);
  void OnGetEdgePositionRsp(common::msg::kinematics::GetEdgePositionRsp data);
  zevs::Socket* socket_;
  uint32_t transaction_id_ = 0;
  struct Transaction {
    uint32_t transaction_id{};
    OnGetSlidesPosition on_get_slides_position;
    OnGetSlidesStatus on_get_slides_status;
    OnGetWeldAxisData on_get_weld_axis_data;
    OnGetEdgePosition on_get_edge_position;
  };
  std::map<uint32_t, Transaction> transactions_;

  uint32_t subscriber_index_{0};
  std::map<uint32_t, std::pair<OnStateChange, EdgeStateSubscriber>> state_change_subscribers_;
};

}  // namespace kinematics
