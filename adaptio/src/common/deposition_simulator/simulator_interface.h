#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "collision-listener-interface.h"
#include "history-provider-interface.h"
#include "point3d.h"
#include "sim-config.h"
#include "torch-interfaces.h"
#include "triangle3d.h"

namespace deposition_simulator {

class ISimulator {
 public:
  virtual ~ISimulator()                                                                                       = 0;
  virtual auto Initialize(SimConfig config) -> void                                                           = 0;
  virtual auto Reset() -> void                                                                                = 0;
  virtual auto CreateSimConfig() -> SimConfig                                                                 = 0;
  virtual auto UpdateTorchPosition(Point3d& torchpos_macs) -> void                                            = 0;
  virtual auto GetTorchPosition(CoordinateSystem ref_system) -> Point3d                                       = 0;
  virtual auto GetTotalDriftFromStart() const -> double                                                       = 0;
  virtual auto UpdateTravelSpeed(double travel_speed) -> void                                                 = 0;
  virtual auto GetAbwPoints(CoordinateSystem ref_system) const -> std::vector<std::optional<Point3d>>         = 0;
  virtual auto GetSliceInTorchPlane(CoordinateSystem ref_system) const -> std::vector<std::optional<Point3d>> = 0;
  virtual auto GetLatestDepositedSlice(CoordinateSystem ref_system) const -> std::vector<Point3d>             = 0;
  virtual auto RunWithRotation(double delta_angle, double bead_radius) -> void                                = 0;
  virtual auto Rotate(double delta_angle) -> void                                                             = 0;
  virtual auto AddSingleWireTorch(double wire_diam, double initial_wire_feed_speed)
      -> std::shared_ptr<ISingleWireTorch>                                                                   = 0;
  virtual auto AddTwinTorch(double wire_diam, double initial_wire_feed_speed) -> std::shared_ptr<ITwinTorch> = 0;
  virtual auto SwitchToRollerBed(double axle_sep, double wheel_sep, double drift_stop_offset) -> void        = 0;
  virtual auto SwitchToChuck() -> void                                                                       = 0;
  virtual auto TouchLeftWall(double stickout) -> std::optional<Point3d>                                      = 0;
  virtual auto TouchRightWall(double stickout) -> std::optional<Point3d>                                     = 0;
  virtual auto RegisterCollisionListener(std::shared_ptr<ICollisionListener> listener) -> void               = 0;
  virtual auto GetWeldObjectMesh() const -> std::vector<Triangle3d>                                          = 0;
};

auto CreateSimulator() -> std::unique_ptr<ISimulator>;
// auto CreateRollerBed() -> std::shared_ptr<ObjectPositioner>;
// auto CreateRollerChuck(double axle_sep, double wheel_sep, double drift_stop_offset) ->
// std::shared_ptr<ObjectPositioner>;
// auto CreateSingleWireTorch(double wire_diam, double initial_wire_feed_speed) -> std::shared_ptr<ISingleWireTorch>;
// auto CreateTwinTorch(double wire_diam, double initial_wire_feed_speed)  -> std::shared_ptr<ITwinTorch>;

}  // namespace deposition_simulator
