#pragma once

#include "calibration/calibration_manager.h"
#include "common/zevs/zevs_core.h"
#include "coordination/activity_status.h"
#include "joint_geometry/joint_geometry_provider.h"
#include "service.h"

namespace web_hmi {

class WebHmiCalibration : public Service, public calibration::CalibrationObserver {
 public:
  explicit WebHmiCalibration(zevs::CoreSocket* socket, calibration::CalibrationManager* calibration_manager,
                             joint_geometry::JointGeometryProvider* joint_geometry_provider,
                             coordination::ActivityStatus* activity_status);

  // CalibrationObserver
  void LaserTorchCalibrationCompleted(const calibration::LaserTorchCalibration& data) override;
  void LaserTorchCalibrationFailed() override;
  void WeldObjectCalibrationCompleted(const calibration::WeldObjectCalibration& data) override;
  void WeldObjectCalibrationFailed() override;

  // Service
  void OnMessage(const std::string& message_name, const nlohmann::json& payload) override;

 private:
  zevs::CoreSocket* socket_;
  calibration::CalibrationManager* calibration_manager_;
  joint_geometry::JointGeometryProvider* joint_geometry_provider_;
  coordination::ActivityStatus* activity_status_;
};

}  // namespace web_hmi
