#pragma once

#include <functional>

#include "joint_geometry/joint_geometry.h"
#include "tracking/tracking_manager.h"
#include "weld_control/weld_control_types.h"
#include "weld_control/weld_state_observer.h"

namespace weld_control {

class WeldControlObserver {
 public:
  virtual ~WeldControlObserver() = default;

  virtual void OnNotifyHandoverToManual() = 0;
  virtual void OnGrooveDataTimeout()      = 0;
  virtual void OnError()                  = 0;
  virtual void OnGracefulStop()           = 0;
};

class WeldControl {
 public:
  virtual ~WeldControl() = default;

  virtual void JointTrackingStart(const joint_geometry::JointGeometry& joint_geometry,
                                  tracking::TrackingMode joint_tracking_mode, double horizontal_offset,
                                  double vertical_offset)                                                    = 0;
  virtual void JointTrackingUpdate(tracking::TrackingMode joint_tracking_mode, double horizontal_offset,
                                   double vertical_offset)                                                   = 0;
  virtual void AutoBeadPlacementStart(LayerType layer_type)                                                  = 0;
  virtual void AutoBeadPlacementStop()                                                                       = 0;
  virtual void Stop()                                                                                        = 0;
  virtual void SetObserver(WeldControlObserver* observer)                                                    = 0;
  virtual auto GetObserver() const -> WeldControlObserver*                                                   = 0;
  virtual void SubscribeReady(std::function<void(const std::vector<std::pair<Mode, LayerType>>&)> on_update) = 0;
  virtual void ResetGrooveData()                                                                             = 0;
  virtual void AddWeldStateObserver(WeldStateObserver* observer)                                             = 0;
  virtual void SetLongitudinalWeldConstraints(std::optional<double> start_position,
                                              std::optional<double> stop_position)                           = 0;
};
}  // namespace weld_control
