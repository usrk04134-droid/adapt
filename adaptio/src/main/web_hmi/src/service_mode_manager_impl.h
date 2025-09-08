#pragma once

#include <memory>

#include "common/zevs/zevs_core.h"
#include "common/zevs/zevs_socket.h"
#include "coordination/activity_status.h"
#include "joint_geometry/joint_geometry_provider.h"
#include "kinematics/kinematics_client.h"
#include "service.h"
#include "web_hmi/web_hmi.h"
#include "weld_control/weld_control.h"

namespace web_hmi {

class ServiceModeManagerImpl {
 public:
  ServiceModeManagerImpl(zevs::CoreSocket* socket, kinematics::KinematicsClient* kinematics_client,
                         joint_geometry::JointGeometryProvider* join_geometry_provider,
                         weld_control::WeldControl* weld_control, coordination::ActivityStatus* activity_status,
                         WebHmi* web_hmi);

 private:
  void Stop();

  zevs::CoreSocket* socket_;
  kinematics::KinematicsClient* kinematics_client_;
  joint_geometry::JointGeometryProvider* joint_geometry_provider_;
  weld_control::WeldControl* weld_control_;
  coordination::ActivityStatus* activity_status_;
  WebHmi* web_hmi_;

  std::unique_ptr<Service> service_;
};

}  // namespace web_hmi
