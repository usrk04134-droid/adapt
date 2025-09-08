#pragma once

#include "controller/controller_data.h"

namespace controller {

// Implement by controller
class KinematicsServerObserver {
 public:
  virtual ~KinematicsServerObserver() = default;

  virtual void OnAxisOutput(AxisOutput data) = 0;
  virtual void Release()                     = 0;
};

// Use from controller
class KinematicsServer {
 public:
  virtual ~KinematicsServer() = default;

  virtual void OnAxisInput(AxisInput axis)                = 0;
  virtual void OnWeldObjectRadius(double radius)          = 0;
  virtual void OnEdgePositionAvailableStatus(bool status) = 0;
  virtual void OnEdgePosition(double value)               = 0;
};

}  // namespace controller
