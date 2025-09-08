#pragma once

#include "common/zevs/zevs_core.h"
#include "joint_geometry/joint_geometry_provider.h"
#include "service.h"
#include "weld_control/weld_control.h"

namespace web_hmi {

class ServiceModeTracking : public Service, public weld_control::WeldControlObserver {
 public:
  explicit ServiceModeTracking(weld_control::WeldControl* weld_control, zevs::CoreSocket* socket,
                               joint_geometry::JointGeometryProvider* joint_geometry_provider);
  ~ServiceModeTracking() override;

  // Service
  void OnMessage(const std::string& message_name, const nlohmann::json& payload) override;

  // TrackingObserver
  void OnNotifyHandoverToManual() override;
  void OnGrooveDataTimeout() override;
  void OnError() override;
  void OnGracefulStop() override;

 private:
  weld_control::WeldControl* weld_control_;
  weld_control::WeldControlObserver* original_observer_;
  zevs::CoreSocket* socket_;
  joint_geometry::JointGeometryProvider* joint_geometry_provider_;
};

}  // namespace web_hmi
