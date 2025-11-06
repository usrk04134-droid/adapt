#pragma once

#include <doctest/doctest.h>
#include <fmt/format.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <nlohmann/json_fwd.hpp>
#include <numbers>
#include <optional>
#include <vector>

#include "helpers.h"
#include "helpers_abp_parameters.h"
#include "helpers_joint_geometry.h"
#include "point3d.h"
#include "sim-config.h"
#include "test_utils/testlog.h"
#include "weld_system_client/weld_system_types.h"

namespace helpers_simulator {
//
// Constants
//
const double PI{std::numbers::pi};
const double SCALE_MM_PER_M{1000.};
const double SCALE_SEC_PER_MS{1000.};
const double SCALE_CM_PER_M{100.};
const double SCALE_SEC_PER_MIN{60.};
const double WELD_OBJECT_CAL_X_ADJUSTMENT{0.0};  // To be set in the simconfig later?
///
/// Test structs
///
struct TestSimulatorJointGeometry {
  struct JointDefenition {
    double basemetal_thickness_m;
    double chamfer_ang_rad;
    double chamfer_len_m;
    double root_face_m;
    double outer_diameter_m;
    double radial_offset_m;
  };
  JointDefenition left;
  JointDefenition right;
  double root_gap_m;
  int joint_depth_percentage;
  double joint_bottom_curv_radius_m;
  double bead_radians_m;
};

struct TestJointGeometry {
  double upper_joint_width_mm;
  double groove_depth_mm;
  double left_joint_angle_rad;
  double right_joint_angle_rad;
  double left_max_surface_angle_rad;
  double right_max_surface_angle_rad;
  TestSimulatorJointGeometry simulator_joint_geometry;
};

struct TestParameters {
  struct ABPParameters {
    double wall_offset_mm;
    double bead_overlap;
    double step_up_value;
    double k_gain;
    struct HeatInput {
      double min;
      double max;
    };
    HeatInput heat_input;
    struct WeldSystem2Current {
      double min;
      double max;
    };
    WeldSystem2Current weld_system_2_current;
    struct WeldSpeed {
      double min;
      double max;
    };
    WeldSpeed weld_speed;
    double bead_switch_angle;
    std::vector<double> step_up_limits;
    double cap_corner_offset;
    int cap_beads;
    double cap_init_depth;
  };
  ABPParameters abp_parameters;
  struct WeldingParameters {
    double weld_object_diameter_m;
    double weld_object_speed_cm_per_min;
    double stickout_m;
    struct WeldSystemParameters {
      double voltage;
      double current;
      double wire_lin_velocity_mm_per_sec;
      double deposition_rate;
      double heat_input;
      bool twin_wire;
      double wire_diameter_mm;
    };
    WeldSystemParameters weld_system_1;
    WeldSystemParameters weld_system_2;
    bool use_edge_sensor;
  };
  WeldingParameters welding_parameters;
  TestJointGeometry test_joint_geometry;
  struct TestCaseParameters {
    std::list<std::string> expected_bead_operations{"steady", "overlapping", "repositioning", "steady"};
    std::map<int, int> expected_beads_in_layer;
  };
  TestCaseParameters testcase_parameters;
};

///
/// Pre defined joint geometries
///
/// TEST_JOINT_GEOMETRY_WIDE
///  Upper joint width: 57.58 (mm) calc from simulator geometry like this:
///  W = root_gap + 2 * basemetal_thickness * tan(joint_angle)
///  Groove depth: 19.6 (mm) calc from simulator geometry like this:
///  D = basemetal_thickness * joint_depth_percentage
///  Left joint angle: 30 (deg)
///  Right joint angle: 30 (deg)
///  Weld Object diamter:
///    Left: 2.0 m
///    Right: 2.0 m
const TestJointGeometry TEST_JOINT_GEOMETRY_WIDE{
    .upper_joint_width_mm        = 57.58,
    .groove_depth_mm             = 19.6,
    .left_joint_angle_rad        = 0.5236,
    .right_joint_angle_rad       = 0.5236,
    .left_max_surface_angle_rad  = 0.3491,
    .right_max_surface_angle_rad = 0.3491,
    .simulator_joint_geometry    = {.left                       = {.basemetal_thickness_m = 0.049,
                                                                   .chamfer_ang_rad       = 0.0,
                                                                   .chamfer_len_m         = 0.0,
                                                                   .root_face_m           = 0.0,
                                                                   .outer_diameter_m      = 2.0,
                                                                   .radial_offset_m       = 0},
                                    .right                      = {.basemetal_thickness_m = 0.049,
                                                                   .chamfer_ang_rad       = 0.0,
                                                                   .chamfer_len_m         = 0.0,
                                                                   .root_face_m           = 0.0,
                                                                   .outer_diameter_m      = 2.0,
                                                                   .radial_offset_m       = 0},
                                    .root_gap_m                 = 1e-3,
                                    .joint_depth_percentage     = 40,
                                    .joint_bottom_curv_radius_m = 1.0,
                                    .bead_radians_m             = 8e-3}
};
/// TEST_JOINT_GEOMETRY_NARROW
/// The joint_angle has been calculated to give a groove width of 44mm according to the formula
/// W = root_gap + 2 * basemetal_thickness * tan(joint_angle)
///  Upper joint width: 44 (mm)
///  Groove depth: 20.6 (mm)
///  Left joint angle: 23.69 (deg
///  Right joint angle: 23.69 (deg)
///  Weld Object diamter:
///    Left: 2.0 m
///    Right: 2.0 m
const TestJointGeometry TEST_JOINT_GEOMETRY_NARROW{
    .upper_joint_width_mm        = 44.0,
    .groove_depth_mm             = 20.6,
    .left_joint_angle_rad        = 0.4135,
    .right_joint_angle_rad       = 0.4135,
    .left_max_surface_angle_rad  = 0.3491,
    .right_max_surface_angle_rad = 0.3491,
    .simulator_joint_geometry    = {.left                       = {.basemetal_thickness_m = 0.049,
                                                                   .chamfer_ang_rad       = 0.0,
                                                                   .chamfer_len_m         = 0.0,
                                                                   .root_face_m           = 0.0,
                                                                   .outer_diameter_m      = 2.0,
                                                                   .radial_offset_m       = 0},
                                    .right                      = {.basemetal_thickness_m = 0.049,
                                                                   .chamfer_ang_rad       = 0.0,
                                                                   .chamfer_len_m         = 0.0,
                                                                   .root_face_m           = 0.0,
                                                                   .outer_diameter_m      = 2.0,
                                                                   .radial_offset_m       = 0},
                                    .root_gap_m                 = 1e-3,
                                    .joint_depth_percentage     = 40,
                                    .joint_bottom_curv_radius_m = 1.0,
                                    .bead_radians_m             = 8e-3}
};
/// const TestJointGeometry TEST_JOINT_GEOMETRY_DEEP{

///  Upper joint width: 53.2 (mm)
///  Groov depth: 71.4 (mm)
///  Left joint angle: 15 (deg
///  Rigth joint angle: 15 (deg)
///  Weld Object diamter:
///    Left: 2.0 m
///    Right: 2.01 m
const TestJointGeometry TEST_JOINT_GEOMETRY_DEEP{
    .upper_joint_width_mm        = 53.2,
    .groove_depth_mm             = 82.1,
    .left_joint_angle_rad        = 0.2618,
    .right_joint_angle_rad       = 0.2618,
    .left_max_surface_angle_rad  = 0.3491,
    .right_max_surface_angle_rad = 0.3491,
    .simulator_joint_geometry    = {.left                       = {.basemetal_thickness_m = 0.1,
                                                                   .chamfer_ang_rad       = 0.0,
                                                                   .chamfer_len_m         = 0.0,
                                                                   .root_face_m           = 0.010,
                                                                   .outer_diameter_m      = 2.00,
                                                                   .radial_offset_m       = 0},
                                    .right                      = {.basemetal_thickness_m = 0.1,
                                                                   .chamfer_ang_rad       = 0.0,
                                                                   .chamfer_len_m         = 0.0,
                                                                   .root_face_m           = 0.010,
                                                                   .outer_diameter_m      = 2.01,
                                                                   .radial_offset_m       = 0},
                                    .root_gap_m                 = 5e-3,
                                    .joint_depth_percentage     = 80,
                                    .joint_bottom_curv_radius_m = 1.0,
                                    .bead_radians_m             = 10e-3}
};
/// TEST_JOINT_GEOMETRY_U_BEVEL
///  Upper joint width: 23 (mm)
///  Groove depth: 33.0 (mm)
///  Left joint angle: 8.0 (deg
///  Right joint angle: 8.0 (deg)
///  Weld Object diamter:
///    Left: 2.0 m
///    Right: 2.0 m
const TestJointGeometry TEST_JOINT_GEOMETRY_U_BEVEL{
    .upper_joint_width_mm        = 23.0,
    .groove_depth_mm             = 33.0,
    .left_joint_angle_rad        = 0.1396,
    .right_joint_angle_rad       = 0.1396,
    .left_max_surface_angle_rad  = 0.3491,
    .right_max_surface_angle_rad = 0.3491,
    .simulator_joint_geometry    = {.left                       = {.basemetal_thickness_m = 0.078,
                                                                   .chamfer_ang_rad       = 0.0,
                                                                   .chamfer_len_m         = 0.0,
                                                                   .root_face_m           = 0.0,
                                                                   .outer_diameter_m      = 2.0,
                                                                   .radial_offset_m       = 0},
                                    .right                      = {.basemetal_thickness_m = 0.078,
                                                                   .chamfer_ang_rad       = 0.0,
                                                                   .chamfer_len_m         = 0.0,
                                                                   .root_face_m           = 0.0,
                                                                   .outer_diameter_m      = 2.0,
                                                                   .radial_offset_m       = 0},
                                    .root_gap_m                 = 1e-3,
                                    .joint_depth_percentage     = 42,
                                    .joint_bottom_curv_radius_m = 1.0,
                                    .bead_radians_m             = 8e-3}
};

//
// Converters
//
inline auto ConvertM2Mm(double meter) { return meter * SCALE_MM_PER_M; }
inline auto ConvertMm2M(double mm) { return mm / SCALE_MM_PER_M; }
inline auto ConvertSec2Ms(double sec) { return sec * SCALE_SEC_PER_MS; }
inline auto ConvertMmPerS2MPerS(double mm_per_s) { return mm_per_s / SCALE_MM_PER_M; }
inline auto ConvertMPerS2RadPerS(double m_per_s, double radius) { return m_per_s / radius; }
inline auto ConvertRadPerS2MPerS(double rad_per_s, double radius) { return radius * rad_per_s; }
inline auto ConvertCMPerMin2MPerS(double cm_per_min) { return cm_per_min / (SCALE_CM_PER_M * SCALE_SEC_PER_MIN); }
inline auto CalculateStepTimeMs(double weld_object_diameter, double weld_object_speed_cm_per_min, int steps) {
  return ConvertSec2Ms(2.0 * PI * weld_object_diameter / (2 * ConvertCMPerMin2MPerS(weld_object_speed_cm_per_min))) /
         steps;
}
inline auto CalculateHeatInputValue(double voltage, double current, double weld_speed_m_per_s) {
  return voltage * current / (weld_speed_m_per_s * 1000 * 1000);
}
///
/// Functions
///
inline auto CalculateWireSpeedMmPerSec(weld_system::WeldSystemSettings::Method method, double wire_diameter_mm,
                                       double current) {
  // Raw Data
  // Current: [ 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500.]
  // DC plus wire-speed (4mm wire):
  // [11.1167, 14.8333, 18.5333, 22.2333, 25.95, 29.65, 33.35, 37.0667, 40.7667, 44.4833, 48.1833, 51.8833, 55.6]
  // AC wire-speed (4mm wire):
  // [13.9, 18.533, 23.1667, 27.8, 32.4333, 37.0667, 41.7, 46.3333, 50.9667, 55.6, 60.2333, 64.85, 69.4833]
  // Support only 4mm wire
  if (wire_diameter_mm != 4.) {
    return 0.;
  }
  if (method == weld_system::WeldSystemSettings::Method::DC) {
    return 0.03707 * current;
  }
  if (method == weld_system::WeldSystemSettings::Method::AC) {
    return 0.04632 * current;
  }
  return 0.0;
}
inline auto CalculateStandardDeviation(const std::vector<double>& data, double mean) {
  double standard_deviation = 0.0;

  for (const auto& value : data) {
    standard_deviation += pow(value - mean, 2);
  }

  return sqrt(standard_deviation / static_cast<double>(data.size()));
}
inline auto SetSimulatorDefault(deposition_simulator::SimConfig& sim_config, int number_of_steps_per_rev) {
  sim_config.nbr_abw_points          = 7;
  sim_config.total_width             = 0.3;
  sim_config.nbr_slices_per_rev      = number_of_steps_per_rev;
  sim_config.nbr_joint_bottom_points = 10;
}

inline auto SetJointGeometry(TestFixture& fixture, deposition_simulator::SimConfig& sim_config,
                             const TestJointGeometry& test_joint_geometry) {
  // Set simulator joint geometry
  sim_config.joint_def_left.basemetal_thickness =
      test_joint_geometry.simulator_joint_geometry.left.basemetal_thickness_m;
  sim_config.joint_def_left.chamfer_ang    = test_joint_geometry.simulator_joint_geometry.left.chamfer_ang_rad;
  sim_config.joint_def_left.chamfer_len    = test_joint_geometry.simulator_joint_geometry.left.chamfer_len_m;
  sim_config.joint_def_left.groove_ang     = test_joint_geometry.left_joint_angle_rad;
  sim_config.joint_def_left.root_face      = test_joint_geometry.simulator_joint_geometry.left.root_face_m;
  sim_config.joint_def_left.outer_diameter = test_joint_geometry.simulator_joint_geometry.left.outer_diameter_m;
  sim_config.joint_def_left.radial_offset  = test_joint_geometry.simulator_joint_geometry.left.radial_offset_m;

  sim_config.joint_def_right.basemetal_thickness =
      test_joint_geometry.simulator_joint_geometry.right.basemetal_thickness_m;
  sim_config.joint_def_right.chamfer_ang    = test_joint_geometry.simulator_joint_geometry.right.chamfer_ang_rad;
  sim_config.joint_def_right.chamfer_len    = test_joint_geometry.simulator_joint_geometry.right.chamfer_len_m;
  sim_config.joint_def_right.groove_ang     = test_joint_geometry.right_joint_angle_rad;
  sim_config.joint_def_right.root_face      = test_joint_geometry.simulator_joint_geometry.right.root_face_m;
  sim_config.joint_def_right.outer_diameter = test_joint_geometry.simulator_joint_geometry.right.outer_diameter_m;
  sim_config.joint_def_right.radial_offset  = test_joint_geometry.simulator_joint_geometry.right.radial_offset_m;

  sim_config.root_gap                 = test_joint_geometry.simulator_joint_geometry.root_gap_m;
  sim_config.joint_depth_percentage   = test_joint_geometry.simulator_joint_geometry.joint_depth_percentage;
  sim_config.joint_bottom_curv_radius = test_joint_geometry.simulator_joint_geometry.joint_bottom_curv_radius_m;

  // Set adaptio joint geometry
  auto const payload = nlohmann::json({
      {"upper_joint_width_mm",        test_joint_geometry.upper_joint_width_mm       },
      {"groove_depth_mm",             test_joint_geometry.groove_depth_mm            },
      {"left_joint_angle_rad",        test_joint_geometry.left_joint_angle_rad       },
      {"right_joint_angle_rad",       test_joint_geometry.right_joint_angle_rad      },
      {"left_max_surface_angle_rad",  test_joint_geometry.left_max_surface_angle_rad },
      {"right_max_surface_angle_rad", test_joint_geometry.right_max_surface_angle_rad},
  });

  StoreJointGeometryParams(fixture, payload, true);
}

inline auto ConfigLPCS(deposition_simulator::SimConfig& sim_config, double stickout_m, double scanner_mount_angle) {
  // Simulator relationship between tcs and lpcs, translation vector and angle.
  sim_config.lpcs_config.alpha = scanner_mount_angle;
  sim_config.lpcs_config.x     = 0;
  sim_config.lpcs_config.y     = 350e-3;
  sim_config.lpcs_config.z     = -stickout_m;
}

inline auto ConfigOPCS(deposition_simulator::SimConfig& sim_config, double weld_object_diameter_m, double stickout_m) {
  // Simulator relation between MACS and ROCS. Only translation vector, no rotation.
  double torch_clock_ang    = 5.0 * PI / 180.;
  double weld_object_radius = weld_object_diameter_m / 2;
  double stick_out_at_calib = stickout_m;
  sim_config.opcs_config.x  = 0;
  sim_config.opcs_config.y  = -(weld_object_radius + stick_out_at_calib) * sin(torch_clock_ang);
  sim_config.opcs_config.z  = -(weld_object_radius + stick_out_at_calib) * cos(torch_clock_ang);
}

inline auto SetABPParameters(TestFixture& fixture, TestParameters& test_parameters) {
  // Store ABP parameters (don't exist in DepSim)
  auto json_step_up_limits = nlohmann::json::array();

  for (auto const& step_up_limit : test_parameters.abp_parameters.step_up_limits) {
    json_step_up_limits.push_back(step_up_limit);
  }

  auto payload = nlohmann::json({
      {"wallOffset",         test_parameters.abp_parameters.wall_offset_mm   },
      {"beadOverlap",        test_parameters.abp_parameters.bead_overlap     },
      {"stepUpValue",        test_parameters.abp_parameters.step_up_value    },
      {"kGain",              test_parameters.abp_parameters.k_gain           },
      {"heatInput",
       {
           {"min", test_parameters.abp_parameters.heat_input.min},
           {"max", test_parameters.abp_parameters.heat_input.max},
       }                                                                     },
      {"weldSystem2Current",
       {
           {"min", test_parameters.abp_parameters.weld_system_2_current.min},
           {"max", test_parameters.abp_parameters.weld_system_2_current.max},
       }                                                                     },
      {"stepUpLimits",       json_step_up_limits                             },
      {"capCornerOffset",    test_parameters.abp_parameters.cap_corner_offset},
      {"capBeads",           test_parameters.abp_parameters.cap_beads        },
      {"capInitDepth",       test_parameters.abp_parameters.cap_init_depth   },
  });

  if (test_parameters.abp_parameters.weld_speed.min > 0.0) {
    payload["weldSpeed"] = {
        {"min", test_parameters.abp_parameters.weld_speed.min},
        {"max", test_parameters.abp_parameters.weld_speed.max},
    };
  }
  if (test_parameters.abp_parameters.bead_switch_angle > 0.0) {
    payload["beadSwitchAngle"] = test_parameters.abp_parameters.bead_switch_angle;
  }
  StoreABPParams(fixture, payload, true);
}

inline auto ComputeLtcTorchToLaserPlaneDistance(deposition_simulator::LpcsConfig& lpcs_config, double stickout_at_ltc)
    -> double {
  // Compute the distance along MACS y from torch to the laser plane
  // That is, the output from simplified LTC
  Eigen::Vector3d t_3to4 = {lpcs_config.x, lpcs_config.y, lpcs_config.z};
  Eigen::Vector3d axis   = {1.0, 0.0, 0.0};
  Eigen::Matrix3d rot_x  = Eigen::AngleAxisd((-std::numbers::pi / 2) - lpcs_config.alpha, axis).toRotationMatrix();
  axis                   = {0.0, 0.0, 1.0};
  Eigen::Matrix3d rot_z  = Eigen::AngleAxisd(std::numbers::pi, axis).toRotationMatrix();
  Eigen::Matrix3d R_3to4 = (rot_x * rot_z).transpose();

  Eigen::Vector3d q_4 = {0.0, 0.0, 0.0};
  Eigen::Vector3d q_3 = R_3to4 * q_4 + t_3to4;

  Eigen::Vector3d n_4 = {0.0, 0.0, 1.0};
  Eigen::Vector3d n_3 = R_3to4 * n_4 + t_3to4;

  Eigen::Vector3d d_3 = {0.0, 1.0, 0.0};

  Eigen::Vector3d s_3 = {0.0, 0.0, -stickout_at_ltc};

  double t = (q_3 - s_3).dot(n_3) / d_3.dot(n_3);

  return t;
}

inline auto ConvertFromOptionalAbwVector(const std::vector<std::optional<deposition_simulator::Point3d>>& abw_points)
    -> std::vector<deposition_simulator::Point3d> {
  std::vector<deposition_simulator::Point3d> converted;

  for (const auto& abw : abw_points) {
    converted.push_back(abw.value());
  }

  return converted;
}

inline auto GetSliceData(std::vector<deposition_simulator::Point3d>& abws_lpcs, const std::uint64_t time_stamp)
    -> common::msg::scanner::SliceData {
  common::msg::scanner::SliceData slice_data{};

  for (std::size_t i = 0; i < common::msg::scanner::GROOVE_ARRAY_SIZE && i < abws_lpcs.size(); ++i) {
    slice_data.groove[i] = {.x = ConvertM2Mm(abws_lpcs[i].GetX()), .y = ConvertM2Mm(abws_lpcs[i].GetY())};
  }

  const auto& left  = abws_lpcs.front();
  const auto& right = abws_lpcs.back();
  const double delta_x = right.GetX() - left.GetX();
  const double delta_y = right.GetY() - left.GetY();
  const double sample_denominator =
      static_cast<double>(std::max<std::size_t>(common::msg::scanner::LINE_ARRAY_SIZE - 1, 1));

  for (std::size_t i = 0; i < common::msg::scanner::LINE_ARRAY_SIZE; ++i) {
    const double t = static_cast<double>(i) / sample_denominator;
    const double x = left.GetX() + t * delta_x;
    const double y = left.GetY() + t * delta_y;
    slice_data.line[i] = {.x = ConvertM2Mm(x), .y = ConvertM2Mm(y)};
  }

  slice_data.confidence = common::msg::scanner::SliceConfidence::HIGH;
  slice_data.time_stamp = time_stamp;

  return slice_data;
}

inline auto DumpAbw([[maybe_unused]] const std::vector<deposition_simulator::Point3d>& abws) -> void {
  TESTLOG("ABW MACS x=[{:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f}]", abws[0].GetX(), abws[1].GetX(),
          abws[2].GetX(), abws[3].GetX(), abws[4].GetX(), abws[5].GetX(), abws[6].GetX());
  TESTLOG("ABW MACS z=[{:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f} {:.5f}]", abws[0].GetZ(), abws[1].GetZ(),
          abws[2].GetZ(), abws[3].GetZ(), abws[4].GetZ(), abws[5].GetZ(), abws[6].GetZ());
}

}  // End namespace helpers_simulator
