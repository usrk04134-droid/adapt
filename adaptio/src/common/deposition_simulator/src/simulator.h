#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "../point3d.h"
#include "../sim-config.h"
#include "../simulator_interface.h"
#include "coordinate-transformer.h"
#include "cwo.h"
#include "object-positioner.h"
#include "src/line2d.h"
#include "torch-base.h"
#include "torch-interfaces.h"

namespace deposition_simulator {

enum HorizontalDirection { LEFT_SIDE = 0, RIGHT_SIDE = 1 };

struct SliceDefinition {
  double slice_angle{0.0};
  JointDef joint_def_left;
  JointDef joint_def_right;
  double root_gap{0.0};
  double center_line_offset{0.0};
};

class Simulator : public ISimulator {
 private:
  bool initialized_{false};
  SimConfig sim_config_;
  std::vector<std::shared_ptr<TorchBase>> torches_;
  std::shared_ptr<CircularWeldObject> weld_object_;
  std::shared_ptr<CoordinateTransformer> transformer_;
  std::shared_ptr<ObjectPositioner> object_positioner_;
  std::vector<std::shared_ptr<ICollisionListener>> collision_listeners_;

  auto ExtrudeWeldObject() -> void;
  auto AddBottomToSlices() -> void;
  auto ComputeVolumeDepositionRate() const -> double;
  // static auto ComputeAngleIncrement(double from_angle, double to_angle) -> double;
  auto InternalRun(double delta_angle, double bead_radius) -> void;
  auto CheckReadiness() const -> bool;
  auto TouchWall(double stickout, HorizontalDirection direction) -> std::optional<Point3d>;
  auto CheckForCollision(Point3d &target_torchpos_macs) -> std::optional<double>;
  auto CheckForTorchCollision(Point3d &target_torch_pos_macs) -> std::optional<double>;
  auto CheckForWireCollision(Point3d &target_torchpos_macs) -> std::optional<double>;
  auto CheckForVertexCollision(Point3d &start_pos, Point3d &end_pos) -> std::optional<double>;
  auto CreateSliceDefinition(double slice_angle) const -> SliceDefinition;

 public:
  Simulator();
  ~Simulator() override = default;
  auto Initialize(SimConfig config) -> void override;
  auto Reset() -> void override;
  auto GetAbwPoints(CoordinateSystem ref_system) const -> std::vector<std::optional<Point3d>> override;
  auto GetSliceInTorchPlane(CoordinateSystem ref_system) const -> std::vector<Point3d> override;
  auto AddSingleWireTorch(double wire_diam, double initial_wire_feed_speed)
      -> std::shared_ptr<ISingleWireTorch> override;
  auto AddTwinTorch(double wire_diam, double initial_wire_feed_speed) -> std::shared_ptr<ITwinTorch> override;
  auto SwitchToRollerBed(double axle_sep, double wheel_sep, double drift_stop_offset) -> void override;
  auto SwitchToChuck() -> void override;
  auto UpdateTorchPosition(Point3d &torchpos_macs) -> void override;
  auto UpdateTravelSpeed(double travel_speed) -> void override;
  auto GetTorchPosition() -> Point3d override;
  auto GetTotalDriftFromStart() const -> double override;
  auto RunWithRotation(double delta_angle, double bead_radius) -> void override;
  auto Rotate(double delta_angle) -> void override;
  // auto GetAbwPointCoords(int slice_index, std::vector<double> &x_coords, std::vector<double> &y_coords) const ->
  // void;
  auto GetJointSliceCoords(int slice_index, std::vector<double> &x_coords, std::vector<double> &y_coords) const -> void;
  auto CreateSimConfig() -> SimConfig override;
  auto TouchLeftWall(double stickout) -> std::optional<Point3d> override;
  auto TouchRightWall(double stickout) -> std::optional<Point3d> override;
  auto RegisterCollisionListener(std::shared_ptr<ICollisionListener> listener) -> void override;
  auto GetWeldObjectMesh() const -> std::vector<Triangle3d> override;
};

}  // namespace deposition_simulator
