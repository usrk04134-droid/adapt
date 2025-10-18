
// #include <matplot/matplot.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <numbers>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "../../point3d.h"
#include "../../sim-config.h"
#include "../../simulator_interface.h"
#include "src/geometric-helper.h"
#include "torch-interfaces.h"

using deposition_simulator::ISimulator;
using deposition_simulator::MACS;
using deposition_simulator::Point3d;
using deposition_simulator::SimConfig;

// using deposition_simulator::LPCS;

const double PI                  = std::numbers::pi;
const std::string ABW_PATH       = "ABW.csv";
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

auto main() -> int {  // NOLINT(readability*)
  // Create simulator and set up config.
  std::unique_ptr<ISimulator> simulator = deposition_simulator::CreateSimulator();
  SimConfig sim_config                  = simulator->CreateSimConfig();

  // NOLINTBEGIN(*-magic-numbers)
  sim_config.joint_def_left.basemetal_thickness       = 0.1;
  sim_config.joint_def_left.chamfer_ang               = 0.0;
  sim_config.joint_def_left.chamfer_len               = 0.0;
  sim_config.joint_def_left.groove_ang                = PI * 15 / 180.0;
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
  sim_config.joint_def_right.groove_ang          = PI * 15 / 180.0;
  sim_config.joint_def_right.root_face           = 0.010;
  sim_config.joint_def_right.outer_diameter      = 2.0;
  sim_config.joint_def_right.radial_offset       = 3.0e-3;

  sim_config.nbr_abw_points     = 7;
  sim_config.total_width        = 0.3;
  sim_config.nbr_slices_per_rev = 250;
  sim_config.root_gap           = 0.005;

  sim_config.nbr_joint_bottom_points  = 10;
  sim_config.joint_depth_percentage   = 80;
  sim_config.joint_bottom_curv_radius = 0.050;

  sim_config.travel_speed = 20e-3;
  sim_config.drift_speed  = 1e-3;
  // simConfig.wire_feed_speed = 10.0 / 60;
  // simConfig.wire_diameter   = 4e-3;

  sim_config.deviations.insert({
      .position           = 0,
      .deltas_left        = {.delta_groove_ang  = 0.0 * PI / 180.0,
                             .delta_chamfer_ang = 0.0,
                             .delta_chamfer_len = 0.0,
                             .delta_root_face   = 0.0},
      .deltas_right       = {.delta_groove_ang  = 0.0 * PI / 180.0,
                             .delta_chamfer_ang = 0.0,
                             .delta_chamfer_len = 0.0,
                             .delta_root_face   = 0.0},
      .delta_root_gap     = 0.0,
      .center_line_offset = 0.0
  });

  sim_config.deviations.insert({
      .position           = 70,
      .deltas_left        = {.delta_groove_ang  = 2.0 * PI / 180.0,
                             .delta_chamfer_ang = 0.0,
                             .delta_chamfer_len = 0.0,
                             .delta_root_face   = 0.0},
      .deltas_right       = {.delta_groove_ang  = 8.0 * PI / 180.0,
                             .delta_chamfer_ang = 0.0,
                             .delta_chamfer_len = 0.0,
                             .delta_root_face   = 0.0},
      .delta_root_gap     = 0.0,
      .center_line_offset = 0.0
  });

  sim_config.deviations.insert({
      .position           = 120,
      .deltas_left        = {.delta_groove_ang  = 0.0 * PI / 180.0,
                             .delta_chamfer_ang = 0.0,
                             .delta_chamfer_len = 0.0,
                             .delta_root_face   = 0.0},
      .deltas_right       = {.delta_groove_ang  = 0.0 * PI / 180.0,
                             .delta_chamfer_ang = 0.0,
                             .delta_chamfer_len = 0.0,
                             .delta_root_face   = 0.0},
      .delta_root_gap     = 0.0,
      .center_line_offset = -3.0e-3
  });
  sim_config.deviations.insert({
      .position           = 170,
      .deltas_left        = {.delta_groove_ang  = 0.0 * PI / 180.0,
                             .delta_chamfer_ang = 0.0,
                             .delta_chamfer_len = 0.0,
                             .delta_root_face   = 0.0},
      .deltas_right       = {.delta_groove_ang  = 0.0 * PI / 180.0,
                             .delta_chamfer_ang = 0.0,
                             .delta_chamfer_len = 0.0,
                             .delta_root_face   = 0.0},
      .delta_root_gap     = 2.0e-3, //----
      .center_line_offset = 0.0
  });

  sim_config.deviations.insert({
      .position = 220,
      .deltas_left =
          {.delta_groove_ang = 0.0, .delta_chamfer_ang = 0.0, .delta_chamfer_len = 0.0, .delta_root_face = 0.0},
      .deltas_right =
          {.delta_groove_ang = 0.0, .delta_chamfer_ang = 0.0, .delta_chamfer_len = 0.0, .delta_root_face = 0.0},
      .delta_root_gap     = 0.0,
      .center_line_offset = 0.0
  });

  // Relationship between tcs and lpcs, translation vector and angle.
  sim_config.lpcs_config.alpha = 6.0 * std::numbers::pi / 180;
  sim_config.lpcs_config.x     = 0;
  sim_config.lpcs_config.y     = 350e-3;
  sim_config.lpcs_config.z     = -25e-3;

  // Relation between MACS and OPCS. Only translation vector, no rotation.
  // This is not in general the same as relation to ROCS, which will depend on the positioner used.
  const double torch_clock_ang    = 5.0 * std::numbers::pi / 180;
  const double radius             = sim_config.joint_def_left.outer_diameter / 2;
  const double stick_out_at_calib = 25e-3;
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

  simulator->Initialize(sim_config);
  simulator->AddSingleWireTorch(WIRE_DIAM_1, WIRE_FEED_SPEED);  // The non-adative torch
  const std::shared_ptr<deposition_simulator::ISingleWireTorch> adaptive_torch =
      simulator->AddSingleWireTorch(WIRE_DIAM_1, WIRE_FEED_SPEED);

  // Switch to rollerbed if desired. RollerChuck is default.
  if (USE_ROLLER_BED) {
    simulator->SwitchToRollerBed(ROLLER_AXLE_SEP, ROLLER_WHEEL_SEP, ROLLER_STOP_OFFSET);
  }

  // #################################################################
  // In this section we simulate touch sensing with torch
  // #################################################################

  // Get the equivalent of ABW points in slice at torch plane. Can be useful in order to understand
  // where torch is located in relation to the joint.
  std::vector<Point3d> slice_at_torch = simulator->GetSliceInTorchPlane(MACS);
  for (const auto &point : slice_at_torch) {
    std::cout << point.GetX() << ", " << point.GetY() << ", " << point.GetZ() << "\n";
  }

  // This is how to get simulated touch sense points with torch.
  // First position torch at top center point
  const double stickout_at_touch = 15e-3;  // mm
  Eigen::Vector3d center_top     = (slice_at_torch.at(0).ToVec() + slice_at_torch.at(6).ToVec()) / 2;
  Point3d new_torch_pos          = {center_top(0), center_top(1), center_top(2), MACS};
  simulator->UpdateTorchPosition(new_torch_pos);

  // Then touch the left wall with the wire (torch is moved to left wall)
  std::optional<Point3d> touch_point_left = simulator->TouchLeftWall(stickout_at_touch);

  // Check that torch was moved to the wall (minus wire radius)
  Point3d curr_torch_pos = simulator->GetTorchPosition();
  std::cout << "Torch pos left: " << curr_torch_pos.GetX() << ", " << curr_torch_pos.GetY() << ", "
            << curr_torch_pos.GetZ() << "\n";

  // Then touch the right wall with the wire (torch is moved to right wall)
  std::optional<Point3d> touch_point_right = simulator->TouchRightWall(stickout_at_touch);

  // Check that torch was moved to the wall (minus wire radius)
  curr_torch_pos = simulator->GetTorchPosition();
  std::cout << "Torch pos right: " << curr_torch_pos.GetX() << ", " << curr_torch_pos.GetY() << ", "
            << curr_torch_pos.GetZ() << "\n";

  if (touch_point_left.has_value()) {
    std::cout << touch_point_left.value().GetX() << ", " << touch_point_left.value().GetY() << ", "
              << touch_point_left.value().GetZ() << "\n";
  }

  if (touch_point_right.has_value()) {
    std::cout << touch_point_right.value().GetX() << ", " << touch_point_right.value().GetY() << ", "
              << touch_point_right.value().GetZ() << "\n";
  }
  // #################################################################
  // End of touch sense example
  // #################################################################

  // Proceed to setup and preform "welding"
  std::vector<double> x_coords       = {};
  std::vector<double> y_coords       = {};
  const std::vector<double> z_coords = {};

  // Point3d torchpos_macs(TORCH_INIT_X, 0, TORCH_INIT_Z, MACS);
  Point3d torchpos_macs = new_torch_pos;
  simulator->UpdateTorchPosition(new_torch_pos);
  double curr_angle = 0.0;

  const double delta_angle = 2 * std::numbers::pi / NBR_STEPS_PER_REV;
  int beads_in_layer       = 1;
  double x_step            = NAN;
  double x_left            = NAN;
  double x_pos             = NAN;
  double z_pos             = NAN;
  double width             = NAN;
  double depth             = NAN;
  std::vector<std::optional<Point3d>> abws;

  std::ofstream abw_file;
  abw_file.open(ABW_PATH);

  abws = simulator->GetAbwPoints(MACS);
  for (const auto &abw_point : abws) {
    x_coords.push_back(abw_point->GetX());
    y_coords.push_back(abw_point->GetY());

    // std::cout << abw_point.GetX() << "," << abw_point.GetY() << "," << abw_point.GetZ() << ",";
    abw_file << abw_point->GetX() << "," << abw_point->GetY() << "," << abw_point->GetZ() << ",";
    // if (j == 0) std::cout << p.GetX() << "," << p.GetY() << std::endl;
  }
  abw_file << "\n";

  try {
    for (int i = 0; i < MAX_LAYERS; i++)  // Layers
    {
      // Estimate a new torch position
      abws = simulator->GetAbwPoints(MACS);

      // Approximate width and depth of joint
      width = std::abs(abws[5]->GetX() - abws[1]->GetX());  // NOLINT(*magic-numbers)
      depth = std::abs(abws[0]->GetZ() - abws[1]->GetZ());
      z_pos = abws[0]->GetZ() - depth + stick_out_at_calib;
      // x_left = abws[1].GetX();
      for (auto &abw_point : abws) {
        depth = std::max(std::abs(abws[0]->GetZ() - abw_point->GetZ()), depth);
      }

      if (width < WIDTH_OF_BEAD) {
        beads_in_layer = 1;
      } else if (width >= WIDTH_OF_BEAD && width < 2 * WIDTH_OF_BEAD) {
        beads_in_layer = 2;
      } else if (width >= 2 * WIDTH_OF_BEAD && width < 3 * WIDTH_OF_BEAD) {
        beads_in_layer = 3;
      } else if (width >= 3 * WIDTH_OF_BEAD && width < 4 * WIDTH_OF_BEAD) {
        beads_in_layer = 4;
      } else {
        throw std::runtime_error("blää");
      }

      x_step = width / (beads_in_layer - 1);

      for (int k = 0; k < beads_in_layer; k++)  // Beads in layer
      {
        for (int j = 0; j < NBR_STEPS_PER_REV; j++)  // Steps per rev
        {
          curr_angle += delta_angle;

          // Estimate a new torch position
          abws   = simulator->GetAbwPoints(MACS);
          x_left = abws[1]->GetX();

          if (beads_in_layer == 1) {
            x_pos = x_left - width / 2;
          } else {
            x_pos = x_left - x_step * k;
          }

          torchpos_macs = Point3d(x_pos, 0, z_pos, MACS);
          simulator->UpdateTorchPosition(torchpos_macs);
          simulator->RunWithRotation(delta_angle, DEFAULT_BEAD_RADIUS);

          x_coords.clear();
          y_coords.clear();
          // simulator->GetJointSliceCoords(3, x, y);

          // abws = simulator->GetAbwPoints(LPCS);
          abw_file << "Torchpos:" << torchpos_macs.GetX() << "," << torchpos_macs.GetZ() << "\n";
          abws = simulator->GetAbwPoints(MACS);
          for (const auto &abw_point : abws) {
            x_coords.push_back(abw_point->GetX());
            y_coords.push_back(abw_point->GetY());

            // std::cout << abw_point.GetX() << "," << abw_point.GetY() << "," << abw_point.GetZ() << ",";
            abw_file << abw_point->GetX() << "," << abw_point->GetY() << "," << abw_point->GetZ() << ",";
            // if (j == 0) std::cout << p.GetX() << "," << p.GetY() << std::endl;
          }

          // std::cout << "\n";
          abw_file << "\n";
          // fig2d->current_axes()->plot(x, y, "-o")->color(matplot::color::blue);

          // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }
    }
  } catch (...) {
    std::cout << "Simulation failed before completion.\n";
  }

  abw_file.close();

  abw_file.close();

  return 0;
}
