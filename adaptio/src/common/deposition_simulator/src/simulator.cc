#include "simulator.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <vector>

#include "../point3d.h"
#include "../sim-config.h"
#include "../simulator_interface.h"
#include "../torch-interfaces.h"
#include "collision-listener-interface.h"
#include "coordinate-transformer.h"
#include "cwo.h"
#include "geometric-helper.h"
#include "joint-slice.h"
#include "plane3d.h"
#include "point2d.h"
#include "roller-bed.h"
#include "roller-chuck.h"
#include "single-wire-torch.h"
#include "src/line2d.h"
#include "triangle3d.h"
#include "twin-torch.h"

namespace deposition_simulator {

using deposition_simulator::ISimulator;
using deposition_simulator::Point3d;
using deposition_simulator::SimConfig;
using deposition_simulator::Simulator;

const double PI                      = std::numbers::pi;
const double PERCENT_FRACTION        = 0.01;
const int MIN_JOINT_DEPTH_PERCENTAGE = 0;
const int MAX_JOINT_DEPTH_PERCENTAGE = 100;
const double COLLISION_MARGIN        = 0.0001;

auto Simulator::ExtrudeWeldObject() -> void {
  const double angle_step = 2.0 * PI / this->sim_config_.nbr_slices_per_rev;  // NOLINT
  double slice_angle      = 0.0;
  SliceDefinition slice_def;
  JointSlice new_slice;

  for (int i = 0; i < this->sim_config_.nbr_slices_per_rev; i++) {
    slice_def = CreateSliceDefinition(slice_angle);

    new_slice = JointSlice(slice_def.joint_def_left, slice_def.joint_def_right, slice_angle, slice_def.root_gap,
                           this->sim_config_.total_width, slice_def.center_line_offset);

    // TODO(zachjz): Add deviations to slice if applicable for the current slice angle.
    this->weld_object_->PushSlice(new_slice);
    slice_angle += angle_step;
  }
}

auto Simulator::CreateSliceDefinition(double slice_angle) const -> SliceDefinition {
  // SliceDefinition new_def {.slice_angle = slice_angle,
  //                         .joint_def_left = sim_config_.joint_def_left,
  //                         .joint_def_right = sim_config_.joint_def_right,
  //                         .root_gap = sim_config_.root_gap,
  //                         .center_line_offset = 0.0
  //                       };

  SliceDefinition new_def;
  new_def.slice_angle        = slice_angle;
  new_def.joint_def_left     = sim_config_.joint_def_left;
  new_def.joint_def_right    = sim_config_.joint_def_right;
  new_def.root_gap           = sim_config_.root_gap;
  new_def.center_line_offset = 0.0;

  if (sim_config_.deviations.size() < 2) {
    return new_def;
  }

  JointDeviation preceeding;
  JointDeviation subsequent;

  bool found_preceeding{false};
  bool found_subsequent{false};

  for (const auto &dev : this->sim_config_.deviations) {
    if (dev.GetSliceAngle() <= slice_angle) {
      preceeding       = dev;
      found_preceeding = true;
    } else {
      subsequent       = dev;
      found_subsequent = true;
      break;
    }
  }

  // Edge case when slice_angle is between last deviation and 2PI
  if (!found_subsequent) {
    subsequent = *(this->sim_config_.deviations.begin());
  }

  // Edge case when slice_angle is between 0 and first deviation
  if (!found_preceeding) {
    // preceeding = *(std::next(this->sim_config_.deviations.begin(), this->sim_config_.deviations.size() - 1));
    preceeding = *(this->sim_config_.deviations.rbegin());
  }

  // Compute angle intervals for interpolation across 2PI boundary.
  double eval_ang{NAN};
  double delta_ang{NAN};

  if (preceeding.GetSliceAngle() > subsequent.GetSliceAngle()) {
    eval_ang  = slice_angle < subsequent.GetSliceAngle() ? (2 * PI) - preceeding.GetSliceAngle() + slice_angle
                                                         : slice_angle - preceeding.GetSliceAngle();
    delta_ang = (2 * PI) - preceeding.GetSliceAngle() + subsequent.GetSliceAngle();
  } else {
    eval_ang  = slice_angle - preceeding.GetSliceAngle();
    delta_ang = subsequent.GetSliceAngle() - preceeding.GetSliceAngle();
  }
  // Piece-wise linear interpolation between closest jointdefs
  // Left side deviation adjustment
  new_def.joint_def_left.chamfer_ang +=
      preceeding.deltas_left.delta_chamfer_ang +
      (subsequent.deltas_left.delta_chamfer_ang - preceeding.deltas_left.delta_chamfer_ang) * eval_ang / delta_ang;
  new_def.joint_def_left.chamfer_len +=
      preceeding.deltas_left.delta_chamfer_len +
      (subsequent.deltas_left.delta_chamfer_len - preceeding.deltas_left.delta_chamfer_len) * eval_ang / delta_ang;
  new_def.joint_def_left.groove_ang +=
      preceeding.deltas_left.delta_groove_ang +
      (subsequent.deltas_left.delta_groove_ang - preceeding.deltas_left.delta_groove_ang) * eval_ang / delta_ang;
  new_def.joint_def_left.root_face +=
      preceeding.deltas_left.delta_root_face +
      (subsequent.deltas_left.delta_root_face - preceeding.deltas_left.delta_root_face) * eval_ang / delta_ang;

  // Right side deviation adjustment
  new_def.joint_def_right.chamfer_ang +=
      preceeding.deltas_right.delta_chamfer_ang +
      (subsequent.deltas_right.delta_chamfer_ang - preceeding.deltas_right.delta_chamfer_ang) * eval_ang / delta_ang;
  new_def.joint_def_right.chamfer_len +=
      preceeding.deltas_right.delta_chamfer_len +
      (subsequent.deltas_right.delta_chamfer_len - preceeding.deltas_right.delta_chamfer_len) * eval_ang / delta_ang;
  new_def.joint_def_right.groove_ang +=
      preceeding.deltas_right.delta_groove_ang +
      (subsequent.deltas_right.delta_groove_ang - preceeding.deltas_right.delta_groove_ang) * eval_ang / delta_ang;
  new_def.joint_def_right.root_face +=
      preceeding.deltas_right.delta_root_face +
      (subsequent.deltas_right.delta_root_face - preceeding.deltas_right.delta_root_face) * eval_ang / delta_ang;

  // Non-sided deviation adjustment
  new_def.center_line_offset = preceeding.center_line_offset +
                               (subsequent.center_line_offset - preceeding.center_line_offset) * eval_ang / delta_ang;
  new_def.root_gap = sim_config_.root_gap + preceeding.delta_root_gap +
                     (subsequent.delta_root_gap - preceeding.delta_root_gap) * eval_ang / delta_ang;

  return new_def;
}

auto Simulator::Initialize(SimConfig config) -> void {
  if (config.joint_depth_percentage < MIN_JOINT_DEPTH_PERCENTAGE ||
      config.joint_depth_percentage > MAX_JOINT_DEPTH_PERCENTAGE) {
    throw std::invalid_argument("Joint depth must be between 0 and 100 percent.");
  }

  // TODO(zachjz): Check that jointdefs are internally consistent
  this->sim_config_  = config;
  this->weld_object_ = std::make_shared<CircularWeldObject>(config);
  this->transformer_ = std::make_shared<CoordinateTransformer>(sim_config_.lpcs_config, sim_config_.opcs_config);

  if (this->object_positioner_ == nullptr) {
    const std::shared_ptr<RollerChuck> chuck_norris =
        std::make_shared<RollerChuck>(this->weld_object_, this->transformer_);
    this->object_positioner_ = chuck_norris;  // Default, use chuck
  } else {
    this->object_positioner_->SetCoordinateTransformer(this->transformer_);
    this->object_positioner_->SetWeldObject(this->weld_object_);
  }

  this->ExtrudeWeldObject();
  this->object_positioner_->PositionObject(NAN, 0.0);
  this->AddBottomToSlices();
  this->initialized_ = true;
}

auto Simulator::Reset() -> void {
  this->initialized_ = false;
  this->weld_object_->Reset();
  this->torches_.clear();
}

auto Simulator::AddBottomToSlices() -> void {
  // TODO(zachjz): The calculation of joint depth must be reviewed since the base metals can also be offset i thickness
  // direction.
  double max_joint_depth   = sim_config_.joint_def_left.basemetal_thickness;
  max_joint_depth          = (max_joint_depth > sim_config_.joint_def_right.basemetal_thickness)
                                 ? sim_config_.joint_def_right.basemetal_thickness
                                 : max_joint_depth;
  const double joint_depth = this->sim_config_.joint_depth_percentage * max_joint_depth * PERCENT_FRACTION;

  this->weld_object_->AddJointBottom(sim_config_.joint_bottom_curv_radius, joint_depth,
                                     sim_config_.nbr_joint_bottom_points);
}

auto Simulator::CreateSimConfig() -> SimConfig {
  // return this->simConfig;
  // TODO(zachjz): Setup a reasonable SimConfig to return
  return {};
}

auto Simulator::CheckReadiness() const -> bool {
  if (!initialized_) {
    return false;
  }
  if (this->torches_.empty()) {
    return false;
  }

  return true;
}

// void Simulator::SetSimConfig(SimConfig config)
// {
//   this->initialized = false;

//   if (config.jointDepthPercentage < 0 || config.jointDepthPercentage > 100)
//     throw std::invalid_argument("Joint depth must be between 0 and 100 percent.");

//   //TODO: Check that jointdefs are internally consistent

//   this->simConfig = config;
//   this->transformer = CoordinateTransformer(simConfig.lpcsConfig, simConfig.macsConfig);
// }

auto Simulator::AddSingleWireTorch(double wire_diam, double initial_wire_feed_speed)
    -> std::shared_ptr<ISingleWireTorch> {
  std::shared_ptr<SingleWireTorch> torch_ptr = std::make_shared<SingleWireTorch>(wire_diam, initial_wire_feed_speed);
  this->torches_.push_back(torch_ptr);
  return torch_ptr;
}

auto Simulator::AddTwinTorch(double wire_diam, double initial_wire_feed_speed) -> std::shared_ptr<ITwinTorch> {
  std::shared_ptr<TwinTorch> torch_ptr = std::make_shared<TwinTorch>(wire_diam, initial_wire_feed_speed);
  this->torches_.push_back(torch_ptr);
  return torch_ptr;
}
auto Simulator::SwitchToRollerBed(double axle_sep, double wheel_sep, double drift_stop_offset) -> void {
  const std::shared_ptr<RollerBed> rollerbed = std::make_shared<RollerBed>(weld_object_, transformer_);
  rollerbed->SetRollerBedGeometry(axle_sep, wheel_sep, drift_stop_offset);
  rollerbed->PositionObject(0.0, 0.0);
  this->object_positioner_ = rollerbed;
}

auto Simulator::SwitchToChuck() -> void {
  this->object_positioner_ = std::make_shared<RollerChuck>(weld_object_, transformer_);
  this->object_positioner_->PositionObject(NAN, 0.0);
}

auto Simulator::UpdateTravelSpeed(double travel_speed) -> void {
  if (!CheckReadiness()) {
    throw std::runtime_error("Simulator has not been initilized.");
  }
  this->sim_config_.travel_speed = travel_speed;
}

auto Simulator::UpdateTorchPosition(Point3d &torchpos_macs) -> void {
  if (!CheckReadiness()) {
    throw std::runtime_error("Simulator has not been initilized.");
  }

  if (torchpos_macs.GetRefSystem() != MACS) {
    throw std::invalid_argument("Torch position is not defined w.r.t MACS");
  }
  Point3d new_torchpos_macs  = {torchpos_macs.GetX(), torchpos_macs.GetY(), torchpos_macs.GetZ(), MACS};
  Point3d curr_torchpos_macs = transformer_->GetTorchPos(MACS);

  if (!sim_config_.ignore_collisions) {
    // Determine if the desired torch movement will result in collision
    auto max_free_movement = CheckForCollision(new_torchpos_macs);

    if (max_free_movement.has_value()) {
      // Collision will occur. Compute new torch position as the collision point.
      // Basically simulating that the slide motors will break due to excessive torque.
      const Eigen::Vector3d move_vec     = new_torchpos_macs.ToVec() - curr_torchpos_macs.ToVec();
      const double move_dist             = max_free_movement.value() - COLLISION_MARGIN;
      const Eigen::Vector3d new_torchpos = curr_torchpos_macs.ToVec() + move_vec.normalized() * move_dist;
      new_torchpos_macs                  = {new_torchpos(0), new_torchpos(1), new_torchpos(2), MACS};

      // Notify any collision listeners
      for (const auto &listener : collision_listeners_) {
        listener->OnTorchCollision(new_torchpos_macs);
      }
    }
  }

  // this->torchPos_macs = torchPos_macs;
  this->transformer_->SetTorchPos(new_torchpos_macs);
}

auto Simulator::GetTorchPosition(CoordinateSystem ref_system) -> Point3d {
  return this->transformer_->GetTorchPos(ref_system);
}

auto Simulator::GetAbwPoints(CoordinateSystem ref_system) const -> std::vector<std::optional<Point3d>> {
  if (!CheckReadiness()) {
    throw std::runtime_error("Simulator has not been initilized.");
  }

  const Point3d lpcs_origin_lpcs = Point3d(0, 0, 0, LPCS);
  const Point3d lpcs_origin_rocs = this->transformer_->Transform(lpcs_origin_lpcs, ROCS);
  const Plane3d lp_lpcs          = Plane3d(Vector3d(0, 0, 1), {0, 0, 0, LPCS});
  const Plane3d lp_rocs          = this->transformer_->Transform(lp_lpcs, ROCS);

  return InternalGetSlicePointsInPlane(ref_system, lp_rocs, lpcs_origin_rocs);
}

auto Simulator::GetLatestDepositedSlice(CoordinateSystem ref_system) const -> std::vector<Point3d> {
  if (!CheckReadiness()) {
    throw std::runtime_error("Simulator has not been initilized.");
  }

  Point3d torch_tip_macs = this->transformer_->GetTorchPos(MACS);
  Point3d wire_tip_macs{torch_tip_macs.GetX(), torch_tip_macs.GetY(),
                        torch_tip_macs.GetZ() /*- this->sim_config_.target_stickout*/, MACS};
  Point3d wire_tip_rocs           = this->transformer_->Transform(wire_tip_macs, ROCS);
  const double wire_tip_angle     = std::atan2(-wire_tip_rocs.GetY(), wire_tip_rocs.GetZ());
  wire_tip_macs                   = this->transformer_->Transform(wire_tip_rocs, MACS);
  std::vector<Point3d> slice_rocs = this->weld_object_->GetLatestDepositedSlice(wire_tip_angle);

  // Change basis of points to ref_system
  std::vector<Point3d> slice_in_ref_sys;
  Point3d tmp_ref;
  for (const auto &slice_point : slice_rocs) {
    tmp_ref = this->transformer_->Transform(slice_point, ref_system);
    slice_in_ref_sys.push_back(tmp_ref);
  }

  return slice_in_ref_sys;
}

auto Simulator::InternalGetSlicePointsInPlane(CoordinateSystem ref_system, const Plane3d &slice_plane_rocs,
                                              const Point3d &filter_point_rocs) const
    -> std::vector<std::optional<Point3d>> {
  const std::vector<std::optional<Point3d>> abw_rocs =
      this->weld_object_->GetAbwPointsInPlane(slice_plane_rocs, filter_point_rocs, true);
  std::vector<std::optional<Point3d>> abws_in_ref_sys;
  Point3d tmp_ref;
  for (const auto &abw_point : abw_rocs) {
    if (abw_point) {
      tmp_ref = this->transformer_->Transform(abw_point.value(), ref_system);
      abws_in_ref_sys.push_back(tmp_ref);
    } else {
      abws_in_ref_sys.push_back({});
    }
  }

  return abws_in_ref_sys;
}

auto Simulator::ComputeVolumeDepositionRate() const -> double {
  double vol_dep_rate = 0.0;
  for (const auto &torch : this->torches_) {
    vol_dep_rate += torch->GetVolumeDepositionRate();
  }

  return vol_dep_rate;
}

// Rotates the weld object and deposits weld metal in each slice that passes the torch.
// Rotation angle is defined as rotation around the symmetry axis of the weld object.
auto Simulator::RunWithRotation(double delta_angle, double bead_radius) -> void {  // NOLINT(*easily-swappable-*)
  if (!CheckReadiness()) {
    throw std::runtime_error("Simulator has not been initilized.");
  }
  InternalRun(delta_angle, bead_radius);

  // const Point3d torchpos_clcs = transformer_.GetTorchPos(CLCS);
  //  Project torch position to slice planes.
  // const Point2d torchpos_slice = transformer_.ProjectToSlicePlane(torchpos_clcs);
  //  Positive rotation from CLCS z-axis
  // const double torch_clock_angle = (2 * PI) - atan2(torchpos_clcs.GetY(), torchpos_clcs.GetZ());
  //  const double vol_dep_rate      = ComputeVolumeDepositionRate();  // m3/s, sum for all torches
  //  const double bead_area         = vol_dep_rate / sim_config_.travel_speed;
  //  sim_config_.wire_feed_speed * std::numbers::pi * pow(sim_config_.wire_diameter / 2, 2) / sim_config_.travel_speed;
  //  TODO(zachjz): Determine if this is to be set or calculated.
  // const double stickout = this->sim_config_.target_stickout;  // 20e-3;

  // this->weld_object_.AddDeposit(torchpos_slice, torch_clock_angle, delta_angle, bead_area, bead_radius, stickout);
  // this->transformer_.IncrWeldObjectRotation(delta_angle); //Move into object_positioner
}

auto Simulator::Rotate(double delta_angle) -> void {
  if (!CheckReadiness()) {
    throw std::runtime_error("Simulator has not been initilized.");
  }

  InternalRun(delta_angle, NAN);
}

auto Simulator::InternalRun(double delta_angle, double bead_radius) -> void {  // NOLINT(*easily-swappable-*)

  // TODO(zachjz): In this method we must also check for collision for each rotation step/slice.

  const bool make_deposit = !std::isnan(bead_radius);

  const double vol_dep_rate = ComputeVolumeDepositionRate();  // m3/s, sum for all torches
  const double bead_area    = vol_dep_rate / sim_config_.travel_speed;

  double accum_rotation             = 0.0;
  const double slice_to_slice_angle = 2 * PI / sim_config_.nbr_slices_per_rev;
  double drift_increment            = slice_to_slice_angle * sim_config_.drift_speed;

  const double initial_torch_plane_angle = this->weld_object_->GetTorchPlaneAngle();
  // TODO(zachjz): Probably change fmod to helpers::pmod and remove condition following.
  double target_torch_plane_angle = std::fmod(initial_torch_plane_angle - delta_angle, (2 * PI));
  if (target_torch_plane_angle < 0) {
    target_torch_plane_angle = (2 * PI) + target_torch_plane_angle;
  }
  // std::cout << "Initial:" << initial_torch_plane_angle << "\n";
  // std::cout << "Delta:" << delta_angle << "\n";
  // std::cout << "Target:" << target_torch_plane_angle << "\n";

  // Determine wire tip in slice CS
  Point3d torchpos_macs = transformer_->GetTorchPos(MACS);
  Point3d wire_tip_macs{torchpos_macs.GetX(), torchpos_macs.GetY(), torchpos_macs.GetZ() - sim_config_.target_stickout,
                        MACS};
  Point2d wire_tip_slice = transformer_->ProjectToSlicePlane(wire_tip_macs);
  // Determine torch position in slice CS
  Point3d torchpos_rocs  = transformer_->GetTorchPos(ROCS);
  Point2d torchpos_slice = transformer_->ProjectToSlicePlane(torchpos_rocs);

  double target_radial_stickout = (wire_tip_slice.ToVector() - torchpos_slice.ToVector()).norm();

  JointSlice *maybe_next_slice = this->weld_object_->MoveToPrevSlice();
  // maybe_next_slice->GetSliceAngle());
  const double angle_incr = deposition_simulator::helpers::ComputeNegAngleIncrement(initial_torch_plane_angle,
                                                                                    maybe_next_slice->GetSliceAngle());
  // double angle_incr = maybe_next_slice->GetSliceAngle() - initial_torch_plane_angle;
  accum_rotation += std::abs(angle_incr);
  // std::cout << "Increment:" << angle_incr << "\n";
  // std::cout << "Next slice:" << maybe_next_slice->GetSliceAngle() << "\n";
  // std::cout << "Accum:" << accum_rotation << "\n";

  // Iterate over all slices affected by the delta_angle rotation and add deposit to slices
  while (accum_rotation <= delta_angle) {
    // Position object w.r.t. to new torch plane to get correct torchpos
    this->object_positioner_->PositionObject(maybe_next_slice->GetSliceAngle(), drift_increment);

    if (make_deposit) {
      // Make deposition with the new torch position
      torchpos_rocs  = transformer_->GetTorchPos(ROCS);
      torchpos_slice = transformer_->ProjectToSlicePlane(torchpos_rocs);
      maybe_next_slice->AddBead(bead_area, bead_radius, target_radial_stickout, torchpos_slice,
                                sim_config_.use_process_dependent_deposition);
    }
    // Move to next slice
    maybe_next_slice  = this->weld_object_->MoveToPrevSlice();
    accum_rotation   += slice_to_slice_angle;
    //   std::cout << "Next slice:" << maybe_next_slice->GetSliceAngle() << "\n";
    //   std::cout << "Accum:" << accum_rotation << "\n";
  }

  // Move one slice back since if we moved to far. Is usually the case but sometimes
  // we end up "directly" on the final slice.
  if (accum_rotation > delta_angle) {
    JointSlice *last_passed_slice = this->weld_object_->MoveToNextSlice();

    // Position object at target angle. Possibly between mesh slices
    drift_increment = (target_torch_plane_angle - last_passed_slice->GetSliceAngle()) * sim_config_.drift_speed;
    this->object_positioner_->PositionObject(target_torch_plane_angle, drift_increment);
  }
  // Store curr torch plane angle to know where to start next time.
  this->weld_object_->SetTorchPlaneAngle(target_torch_plane_angle);

  // std::cout << "New torch plane angle:" << this->weld_object_->GetTorchPlaneAngle() << "\n";
}

auto Simulator::GetSliceInTorchPlane(CoordinateSystem ref_system) const -> std::vector<std::optional<Point3d>> {
  if (!CheckReadiness()) {
    throw std::runtime_error("Simulator has not been initilized.");
  }
  const Point3d mcs_origin_macs  = Point3d(0, 0, 0, MACS);
  const Point3d mcs_origin_rocs  = this->transformer_->Transform(mcs_origin_macs, ROCS);
  const Plane3d torch_plane_macs = Plane3d(Vector3d(0, 1, 0), {0, 0, 0, MACS});
  const Plane3d torch_plane_rocs = this->transformer_->Transform(torch_plane_macs, ROCS);

  return InternalGetSlicePointsInPlane(ref_system, torch_plane_rocs, mcs_origin_rocs);
}

auto Simulator::TouchLeftWall(double stickout) -> std::optional<Point3d> {
  if (!CheckReadiness()) {
    throw std::runtime_error("Simulator has not been initilized.");
  }

  return TouchWall(stickout, LEFT_SIDE);
}

auto Simulator::TouchRightWall(double stickout) -> std::optional<Point3d> {
  if (!CheckReadiness()) {
    throw std::runtime_error("Simulator has not been initilized.");
  }

  return TouchWall(stickout, RIGHT_SIDE);
}

auto Simulator::TouchWall(double stickout, HorizontalDirection direction) -> std::optional<Point3d> {
  // Compute what the 2d joint slice looks like in the torch plane
  std::vector<std::optional<Point3d>> abws_in_torch_plane = GetSliceInTorchPlane(MACS);

  // Determine where the wire tip is at in relation to the joint in slice
  double wire_radius = this->torches_.at(0)->GetWireDiameter() / 2;
  wire_radius        = (direction == LEFT_SIDE) ? wire_radius : -wire_radius;
  Point3d wire_tip_tcs{wire_radius, 0.0, -stickout, TCS};
  Point3d wire_tip_macs = this->transformer_->Transform(wire_tip_tcs, MACS);

  // Construct horizontal lines from wire tip towards wall
  Eigen::Vector2d direction_macs = {1, 0};
  Line2d horizontal_line{
      {wire_tip_macs.GetX(), wire_tip_macs.GetZ()},
      direction_macs
  };

  // Construct wall line
  int start_idx = (direction == LEFT_SIDE) ? 0 : 6;  // NOLINT
  int end_idx   = (direction == LEFT_SIDE) ? 1 : 5;  // NOLINT
  Line2d wall_line_macs =
      Line2d::FromPoints({abws_in_torch_plane.at(start_idx)->GetX(), abws_in_torch_plane.at(start_idx)->GetZ()},
                         {abws_in_torch_plane.at(end_idx)->GetX(), abws_in_torch_plane.at(end_idx)->GetZ()});

  // Intersect horizontal lines with joint walls (if possible)
  std::unique_ptr<Point2d> int_point = horizontal_line.Intersect(wall_line_macs, false, true);

  if (int_point == nullptr) {
    return {};
  }

  // Touch point could be determined, now move the torch there
  const Point3d touch_point_macs = {int_point->GetX(), 0.0, int_point->GetY(), MACS};
  const Point3d torch_pos_macs   = this->transformer_->GetTorchPos(MACS);
  const double x_dist_to_wall    = touch_point_macs.GetX() - torch_pos_macs.GetX();
  const double x_move            = x_dist_to_wall - wire_radius;
  Point3d new_torch_pos{torch_pos_macs.GetX() + x_move, torch_pos_macs.GetY(), torch_pos_macs.GetZ(), MACS};
  this->UpdateTorchPosition(new_torch_pos);

  return new_torch_pos;
}

Simulator::Simulator() : weld_object_({}) {
  this->object_positioner_ = nullptr;
  // this->object_positioner_ = std::make_shared<RollerChuck>(weld_object_, transformer_);  // Default, use chuck
}

auto Simulator::RegisterCollisionListener(std::shared_ptr<ICollisionListener> listener) -> void {
  this->collision_listeners_.push_back(listener);
}

// Checks if torch or wire (other components?) will hit the weld object during the intended movement.
auto Simulator::CheckForCollision(Point3d &target_torchpos_macs) -> std::optional<double> {
  Eigen::Vector3d move_vec;
  auto max_move_wire  = CheckForWireCollision(target_torchpos_macs);
  auto max_move_torch = CheckForTorchCollision(target_torchpos_macs);

  // Determine which (if any) collision is closest to current torch position. I.e. max distance torch can be moved.
  double shortest_dist = INFINITY;

  if (max_move_wire.has_value()) {
    shortest_dist = std::min(shortest_dist, max_move_wire.value());
  }

  if (max_move_torch.has_value()) {
    shortest_dist = std::min(shortest_dist, max_move_torch.value());
  }

  if (std::isinf(shortest_dist)) {
    return {};
  }

  return shortest_dist;
}

// Checks if the torch will hit the weld object during the intended movement.
// Returns the distance to the collision point.
auto Simulator::CheckForTorchCollision(Point3d &target_torchpos_macs) -> std::optional<double> {
  Point3d curr_torchpos_macs = transformer_->GetTorchPos(MACS);
  // TODO(zachjz): Here we need to introduce a representation of the torch geometry
  // and construct movement lines for each vertex in the surface and check these lines
  // for intersection with all joint lines. So basically a loop here calling CheckForCollision(...)
  // for each vertex movement line.
  auto dist_to_hit = CheckForVertexCollision(curr_torchpos_macs, target_torchpos_macs);

  return dist_to_hit;
}

// Checks if the left or right side of wire will hit the weld object during the intended movement.
// Returns the distance to the collision point.
auto Simulator::CheckForWireCollision(Point3d &target_torchpos_macs) -> std::optional<double> {
  Point3d curr_torchpos_macs = transformer_->GetTorchPos(MACS);

  // Only check first torch for now
  double wire_radius = this->torches_.front()->GetWireDiameter() / 2;

  // Check left side of wire tip
  Point3d curr_wiretip_macs   = {curr_torchpos_macs.GetX() + wire_radius, curr_torchpos_macs.GetY(),
                                 curr_torchpos_macs.GetZ() - sim_config_.target_stickout, MACS};
  Point3d target_wiretip_macs = {target_torchpos_macs.GetX() + wire_radius, target_torchpos_macs.GetY(),
                                 target_torchpos_macs.GetZ() - sim_config_.target_stickout, MACS};

  double max_move_dist = INFINITY;
  auto dist_to_hit     = CheckForVertexCollision(curr_wiretip_macs, target_wiretip_macs);
  if (dist_to_hit.has_value()) {
    max_move_dist = std::min(dist_to_hit.value(), max_move_dist);
  }

  // Check right side of wire tip
  curr_wiretip_macs   = {curr_torchpos_macs.GetX() - wire_radius, curr_torchpos_macs.GetY(),
                         curr_torchpos_macs.GetZ() - sim_config_.target_stickout, MACS};
  target_wiretip_macs = {target_torchpos_macs.GetX() - wire_radius, target_torchpos_macs.GetY(),
                         target_torchpos_macs.GetZ() - sim_config_.target_stickout, MACS};

  dist_to_hit = CheckForVertexCollision(curr_wiretip_macs, target_wiretip_macs);
  if (dist_to_hit.has_value()) {
    max_move_dist = std::min(dist_to_hit.value(), max_move_dist);
  }

  // Eigen::Vector3d move_vec = target_wiretip_macs.ToVec() - curr_torchpos_macs.ToVec();
  // Eigen::Vector3d torchpos_vec = curr_torchpos_macs.ToVec() + move_vec * shortest_dist;
  // Point3d collision_torchpos = {torchpos_vec(0), torchpos_vec(1),torchpos_vec(2),MACS};

  if (std::isinf(max_move_dist)) {
    return {};
  }

  return max_move_dist;
}

// Checks if a line from start_pos to end_pos intersects base material surface at some point.
// Returns the distance to intersection/hit point if existing.
auto Simulator::CheckForVertexCollision(Point3d &start_pos, Point3d &end_pos) -> std::optional<double> {
  Point2d startpos_2d = {start_pos.GetX(), start_pos.GetZ()};
  Point2d endpos_2d   = {end_pos.GetX(), end_pos.GetZ()};
  Line2d movement     = Line2d::FromPoints(startpos_2d, endpos_2d);

  std::vector<std::optional<Point3d>> torch_plane_points = this->GetSliceInTorchPlane(MACS);
  Line2d surface;
  Point2d surface_start;
  Point2d surface_end;
  std::unique_ptr<Point2d> intersection;

  for (int i = 0; i < torch_plane_points.size() - 1; i++) {
    surface_start = {torch_plane_points.at(i)->GetX(), torch_plane_points.at(i)->GetZ()};
    surface_end   = {torch_plane_points.at(i + 1)->GetX(), torch_plane_points.at(i + 1)->GetZ()};
    surface       = Line2d::FromPoints(surface_start, surface_end);

    intersection = surface.Intersect(movement, true, true);

    if (intersection != nullptr) {
      Point3d hitpoint = {intersection->GetX(), 0.0, intersection->GetY(), MACS};
      return (hitpoint.ToVec() - start_pos.ToVec()).norm();
    }
  }

  return {};
}

auto Simulator::GetTotalDriftFromStart() const -> double { return this->object_positioner_->GetTotalDrift(); }

auto Simulator::GetWeldObjectMesh() const -> std::vector<Triangle3d> { return weld_object_->ToTriangleMesh(); }

ISimulator::~ISimulator() = default;

auto CreateSimulator() -> std::unique_ptr<ISimulator> { return std::make_unique<Simulator>(); }
// auto CreateRollerBed() -> std::shared_ptr<ObjectPositioner> {return std::make_shared<RollerBed>();}
// auto CreateRollerChuck() -> std::shared_ptr<ObjectPositioner> {return std::make_shared<RollerChuck>();}
}  // namespace deposition_simulator
