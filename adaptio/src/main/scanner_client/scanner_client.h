#pragma once

#include <optional>

#include "kinematics/kinematics_client.h"
#include "lpcs/lpcs_slice.h"

namespace scanner_client {

class ScannerObserver {
 public:
  virtual ~ScannerObserver() = default;

  virtual void OnScannerStarted(bool started) = 0;
  virtual void OnScannerStopped(bool stopped) = 0;
  virtual void OnScannerDataUpdate(const lpcs::Slice& data, const common::groove::Point& axis_position) = 0;
};

class ScannerClient {
 public:
  virtual ~ScannerClient() = default;

  struct Config {
    double interval;  // in seconds
    enum class ScannerSensitivity { NORMAL, HIGH } sensitivity;
  };

  virtual void Start(const Config& config, const joint_geometry::JointGeometry& joint_geometry) = 0;
  virtual void Update(const struct UpdateData& data)                                           = 0;
  virtual void Stop()                                                                          = 0;
  virtual void ImageLoggingUpdate(const struct ImageLoggingData& data)                         = 0;
  virtual void FlushImageBuffer()                                                              = 0;
  virtual void AddObserver(ScannerObserver* observer)                                          = 0;
};

}  // namespace scanner_client
