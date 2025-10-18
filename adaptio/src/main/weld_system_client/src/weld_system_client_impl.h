#pragma once

#include <prometheus/counter.h>
#include <prometheus/gauge.h>
#include <prometheus/registry.h>

#include <cstdint>
#include <map>

#include "common/messages/weld_system.h"
#include "common/zevs/zevs_socket.h"
#include "weld_system_client/weld_system_client.h"
#include "weld_system_client/weld_system_types.h"

namespace weld_system {

class WeldSystemClientImpl : public WeldSystemClient {
 public:
  explicit WeldSystemClientImpl(zevs::Socket* socket, prometheus::Registry* registry);

  void GetWeldSystemData(WeldSystemId id, OnGetWeldSystemData on_response) override;
  void SetWeldSystemData(WeldSystemId id, WeldSystemSettings data) override;
  auto SubscribeWeldSystemStateChanges(OnStateChange on_state_change) -> uint32_t override;
  void UnSubscribeWeldSystemStateChanges(uint32_t handle) override;

  void WSStateUpdateMetrics(WeldSystemId id, const WeldSystemState& state);
  void WSDataUpdateMetrics(WeldSystemId id, const WeldSystemData& data);

 private:
  void OnGetWeldSystemDataRsp(common::msg::weld_system::GetWeldSystemDataRsp data);
  void OnWeldSystemStateChange(common::msg::weld_system::OnWeldSystemStateChange data);
  void SetupMetrics(prometheus::Registry* registry);

  zevs::Socket* socket_;
  uint32_t transaction_id_ = 0;
  struct Transaction {
    WeldSystemId id;
    uint32_t transaction_id{};
    OnGetWeldSystemData on_get_status;
  };
  std::map<uint32_t, Transaction> transactions_;
  uint32_t subscriber_index_{0};

  std::map<uint32_t, OnStateChange> state_change_subscribers_;

  struct WeldSystemMetrics {
    std::map<WeldSystemState, prometheus::Gauge*> state_gauges;
    prometheus::Gauge* voltage{nullptr};
    prometheus::Gauge* current{nullptr};
    prometheus::Gauge* wire_lin_velocity{nullptr};
    prometheus::Gauge* deposition_rate{nullptr};
    prometheus::Gauge* heat_input{nullptr};
  };

  std::map<WeldSystemId, WeldSystemMetrics> weld_system_metrics_;
};

}  // namespace weld_system
