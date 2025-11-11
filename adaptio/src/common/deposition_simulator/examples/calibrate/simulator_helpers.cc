
#include "simulator_helpers.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include "collision-listener-interface.h"
#include "point3d.h"
#include "sim-config.h"
#include "simulator_interface.h"
#include "src/geometric-helper.h"

using deposition_simulator::ISimulator;
using deposition_simulator::MACS;
using deposition_simulator::Point3d;

namespace cal_example {

const double PI                  = std::numbers::pi;
const std::string ABW_PATH       = "grid_data.csv";
const double WIRE_DIAM_1         = 4e-3;
const double WIRE_FEED_SPEED     = 5.0 / 60;
const double WIDTH_OF_BEAD       = 15e-3;
const int NBR_STEPS_PER_REV      = 205;
const double DEFAULT_BEAD_RADIUS = 25e-3;
const int MAX_LAYERS             = 20;
const double TORCH_INIT_X        = -25e-3;
const double TORCH_INIT_Z        = -80e-3;
const double ROLLER_AXLE_SEP     = 0.5;
const double ROLLER_WHEEL_SEP    = 0.5;
const double ROLLER_STOP_OFFSET  = 0.13;
const bool USE_ROLLER_BED        = false;
const int NBR_GRID_STEPS_X       = 15;
const int NBR_GRID_STEPS_Z       = 15;
const double GRID_STEP_SIZE      = 10e-3;

auto Configure(deposition_simulator::ISimulator *simulator) -> deposition_simulator::SimConfig {
  deposition_simulator::SimConfig sim_config = simulator->CreateSimConfig();
  // NOLINTBEGIN(*-magic-numbers)
  sim_config.joint_def_left.basemetal_thickness       = 0.1;
  sim_config.joint_def_left.chamfer_ang               = 0.0;
  sim_config.joint_def_left.chamfer_len               = 0.0;
  sim_config.joint_def_left.groove_ang                = PI * 30 / 180.0;
  sim_config.joint_def_left.root_face                 = 0.010;
  sim_config.joint_def_left.outer_diameter            = 2.0;
  sim_config.joint_def_left.long_weld_clock_angle     = PI * 90 / 180.0;
  sim_config.joint_def_left.long_weld_height          = 0.005;
  sim_config.joint_def_left.long_weld_rise_percentage = 80;
  sim_config.joint_def_left.long_weld_width           = 0.400;
  sim_config.joint_def_left.radial_offset             = 0.0;  //-5.0e-3;

  sim_config.joint_def_right.basemetal_thickness = 0.1;
  sim_config.joint_def_right.chamfer_ang         = 0.0;  // PI * 10 / 180.0;
  sim_config.joint_def_right.chamfer_len         = 0.0;  // 20e-3;
  sim_config.joint_def_right.groove_ang          = PI * 30 / 180.0;
  sim_config.joint_def_right.root_face           = 0.010;
  sim_config.joint_def_right.outer_diameter      = 2.0;
  sim_config.joint_def_right.radial_offset       = 0.0e-3;

  sim_config.nbr_abw_points     = 7;
  sim_config.total_width        = 0.3;
  sim_config.nbr_slices_per_rev = 1000;
  sim_config.root_gap           = 0.005;
  sim_config.target_stickout    = 25e-3;
  sim_config.ignore_collisions  = false;

  sim_config.nbr_joint_bottom_points  = 10;
  sim_config.joint_depth_percentage   = 80;
  sim_config.joint_bottom_curv_radius = 0.050;

  sim_config.travel_speed = 20e-3;
  // simConfig.wire_feed_speed = 10.0 / 60;
  // simConfig.wire_diameter   = 4e-3;

  // Relationship between tcs and lpcs, translation vector and angle.
  sim_config.lpcs_config.alpha = 20.0 * std::numbers::pi / 180;
  sim_config.lpcs_config.x     = 0;
  sim_config.lpcs_config.y     = 350e-3;
  sim_config.lpcs_config.z     = -25e-3;

  // Relation between MACS and OPCS. Only translation vector, no rotation.
  // This is not in general the same as relation to ROCS, which will depend on the positioner used.
  const double torch_clock_ang    = 5.0 * std::numbers::pi / 180;
  const double radius             = sim_config.joint_def_left.outer_diameter / 2;
  const double stick_out_at_calib = 35e-3;
  // NOLINTEND(*-magic-numbers)

  // Here we create config for Object Positioner Coordinate System
  if (!USE_ROLLER_BED) {
    // For RollerChuck - rotation around symmetry axis of object.
    sim_config.opcs_config.x = 0;
    sim_config.opcs_config.y = -(radius + stick_out_at_calib) * sin(torch_clock_ang);
    sim_config.opcs_config.z = -(radius + stick_out_at_calib) * cos(torch_clock_ang);
  } else {
    // For RollerBed (just make sure MACS origin i approx. stickout above surface)
    const double roller_sep_y = 0.5;  // MACS/OPCS y separation of roller wheel contact points.
    sim_config.opcs_config    = deposition_simulator::helpers::ComputeOpcsConfigForRollerBed(
        torch_clock_ang, radius, stick_out_at_calib, ROLLER_AXLE_SEP);
  }

  return sim_config;
}

auto WriteToFile(std::vector<std::optional<deposition_simulator::Point3d>> &points, std::ofstream &file) -> void {
  int count = 0;
  for (const auto &abw : points) {
    file << abw->ToString();

    count++;

    if (count < points.size()) {
      file << ",";
    }
  }
}

auto WriteLineToFile(std::string name, deposition_simulator::Point3d &point, std::ofstream &file) -> void {
  std::vector<Point3d> points;
  points.push_back(point);

  file << name << ":";
  WriteToFile(points, file);
  file << "\n";
}

auto WriteLineToFile(std::string name, std::vector<std::optional<deposition_simulator::Point3d>> &points,
                     std::ofstream &file) -> void {
  file << name << ":";
  WriteToFile(points, file);
  file << "\n";
}

auto MoveInGrid(ISimulator *simulator) -> void {
  std::ofstream grid_file;
  grid_file.open(ABW_PATH);

  auto collision_listener = std::make_shared<ExampleListener>();
  simulator->RegisterCollisionListener(collision_listener);
  Point3d top_center_torch_pos;
  Point3d initial_torch_pos;
  Point3d center_torch_pos;
  Point3d torch_pos;
  Point3d test_torch_pos;

  std::vector<Point3d> abw_torch_plane = simulator->GetSliceInTorchPlane(MACS);
  WriteLineToFile("ABW torch plane", abw_torch_plane, grid_file);

  // Set initial torch position
  initial_torch_pos = {(abw_torch_plane.at(0).GetX() + abw_torch_plane.at(6).GetX()) / 2,
                       (abw_torch_plane.at(0).GetY() + abw_torch_plane.at(6).GetY()) / 2,
                       (abw_torch_plane.at(0).GetZ() + abw_torch_plane.at(6).GetZ()) / 2 + 15e-3, MACS};

  center_torch_pos = {(abw_torch_plane.at(0).GetX() + abw_torch_plane.at(6).GetX()) / 2, 0.0, 0.0, MACS};

  top_center_torch_pos = {(abw_torch_plane.at(0).GetX() + abw_torch_plane.at(6).GetX()) / 2,
                          (abw_torch_plane.at(0).GetY() + abw_torch_plane.at(6).GetY()) / 2,
                          (abw_torch_plane.at(0).GetZ() + abw_torch_plane.at(6).GetZ()) / 2, MACS};

  // Define grid reference position
  double x_macs = initial_torch_pos.GetX() - (NBR_GRID_STEPS_X * GRID_STEP_SIZE) / 2;
  double z_macs = abw_torch_plane.at(1).GetZ() + 2e-3;

  simulator->UpdateTorchPosition(center_torch_pos);

  simulator->UpdateTorchPosition(top_center_torch_pos);
  std::vector<std::optional<Point3d>> abw_points = simulator->GetAbwPoints(deposition_simulator::LPCS);
  top_center_torch_pos                           = {(abw_points.at(0)->GetX() + abw_points.at(6)->GetX()) / 2,
                                                    (abw_points.at(0)->GetY() + abw_points.at(6)->GetY()) / 2,
                                                    (abw_points.at(0)->GetZ() + abw_points.at(6)->GetZ()) / 2, deposition_simulator::LPCS};
  WriteLineToFile("Top center when torch at ref", top_center_torch_pos, grid_file);

  simulator->UpdateTorchPosition(initial_torch_pos);
  WriteLineToFile("Initial torch pos", initial_torch_pos, grid_file);

  // Do touch sense
  simulator->TouchLeftWall(25e-3);
  torch_pos = simulator->GetTorchPosition(deposition_simulator::MACS);
  WriteLineToFile("Touch left", torch_pos, grid_file);

  simulator->TouchRightWall(25e-3);
  torch_pos = simulator->GetTorchPosition(deposition_simulator::MACS);
  WriteLineToFile("Touch right", torch_pos, grid_file);

  simulator->UpdateTorchPosition(initial_torch_pos);

  for (int iz = 0; iz < NBR_GRID_STEPS_Z; iz++) {
    for (int ix = 0; ix < NBR_GRID_STEPS_X; ix++) {
      std::cout << "Iteration: (" << ix << "," << iz << ")\n";
      double x_coord = (std::fmod(iz, 2) == 0) ? (ix * GRID_STEP_SIZE + x_macs)
                                               : (NBR_GRID_STEPS_X - ix - 1) * GRID_STEP_SIZE + x_macs;
      test_torch_pos = {x_coord, 0, iz * GRID_STEP_SIZE + z_macs, deposition_simulator::MACS};

      // Try to move torch to new position
      simulator->UpdateTorchPosition(test_torch_pos);

      // Check if move resulted in collision
      if (collision_listener->CollisionDetected()) {
        collision_listener->Reset();
        simulator->UpdateTorchPosition(initial_torch_pos);  // Go back to prev
        std::cout << "Collision!\n";
        continue;
      }

      // No collision -> continue to write out observations
      torch_pos = test_torch_pos;

      abw_points = simulator->GetAbwPoints(deposition_simulator::LPCS);
      WriteLineToFile("Observation LPCS", abw_points, grid_file);

      abw_points = simulator->GetAbwPoints(deposition_simulator::MACS);
      WriteLineToFile("Observation MACS", abw_points, grid_file);

      WriteLineToFile("Slide cross", torch_pos, grid_file);
    }
  }

  grid_file.close();
}

auto ExampleListener::CollisionDetected() const -> bool { return has_collided_; }

auto ExampleListener::Reset() -> void { has_collided_ = false; }

auto ExampleListener::OnTorchCollision(deposition_simulator::Point3d &hit_point) -> void { has_collided_ = true; }

}  // Namespace cal_example

deposition_simulator::ICollisionListener::~ICollisionListener() = default;
