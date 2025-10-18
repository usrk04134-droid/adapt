#pragma once
#include <map>

#include "common/messages/weld_system.h"
#include "common/zevs/zevs_socket.h"
#include "controller/controller_data.h"
#include "weld_system_server.h"

namespace controller {

class WeldSystemServerImpl : public WeldSystemServer {
 public:
  explicit WeldSystemServerImpl(zevs::Socket* socket, WeldSystemServerObserver* observer);

  void OnPowerSourceInput(uint32_t index, PowerSourceInput const& data) override;

 private:
  zevs::Socket* socket_;
  WeldSystemServerObserver* observer_;
  bool state_change_subscriber_{false};

  std::map<uint32_t, PowerSourceInput> weld_systems_;
  void OnGetWeldSystemData(common::msg::weld_system::GetWeldSystemData data);
  void OnSetWeldSystemSettings(common::msg::weld_system::SetWeldSystemSettings data);
  void SendStateChange(uint32_t index, const PowerSourceInput& data);
  void SubscribeWeldSystemStateChanges(common::msg::weld_system::SubscribeWeldSystemStateChanges data);
  void UnSubscribeWeldSystemStateChanges(common::msg::weld_system::UnSubscribeWeldSystemStateChanges data);
};

}  // namespace controller
