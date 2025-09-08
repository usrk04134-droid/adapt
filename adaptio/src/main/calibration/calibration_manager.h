#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include "calibration_types.h"
#include "joint_geometry/joint_geometry.h"

namespace calibration {

class CalibrationObserver {
 public:
  virtual ~CalibrationObserver() = default;

  virtual void LaserTorchCalibrationCompleted(const calibration::LaserTorchCalibration& data) = 0;
  virtual void LaserTorchCalibrationFailed()                                                  = 0;
  virtual void WeldObjectCalibrationCompleted(const calibration::WeldObjectCalibration& data) = 0;
  virtual void WeldObjectCalibrationFailed()                                                  = 0;
};

class CalibrationManager {
 public:
  CalibrationManager()                                     = default;
  virtual ~CalibrationManager()                            = default;
  virtual void SetObserver(CalibrationObserver* observer)  = 0;
  virtual auto GetObserver() const -> CalibrationObserver* = 0;

  virtual void StartCalibrateLaserToTorch(const joint_geometry::JointGeometry& calibration_joint_geometry,
                                          double offset, double angle, double stickout)                    = 0;
  virtual void StartCalibrateLaserToTorch(const joint_geometry::JointGeometry& calibration_joint_geometry) = 0;
  virtual void StartCalibrateWeldObject(const joint_geometry::JointGeometry& calibration_joint_geometry, double radius,
                                        double stickout)                                                   = 0;
  virtual void StartCalibrateWeldObject(const joint_geometry::JointGeometry& calibration_joint_geometry)   = 0;
  virtual void Stop()                                                                                      = 0;
  virtual auto GetLaserToTorchCalibration() const -> std::optional<LaserTorchCalibration>                  = 0;
  virtual auto SetLaserToTorchCalibration(const LaserTorchCalibration& data) -> bool                       = 0;
  virtual auto GetWeldObjectCalibration() const -> std::optional<WeldObjectCalibration>                    = 0;
  virtual auto SetWeldObjectCalibration(const WeldObjectCalibration& data) -> bool                         = 0;
};

using CalibrationManagerPtr = std::shared_ptr<CalibrationManager>;

}  // namespace calibration
