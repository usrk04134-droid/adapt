#pragma once

#include <cmath>
#include <set>
#include <vector>
namespace deposition_simulator {

struct JointDef {
  double basemetal_thickness{0.0};
  double groove_ang{0.0};
  double chamfer_ang{0.0};
  double chamfer_len{0.0};
  double root_face{0.0};
  double outer_diameter{0.0};
  double radial_offset{0.0};
  double long_weld_clock_angle{NAN};  // Clock pos/angle of longitudinal weld
  double long_weld_height{0.0};
  double long_weld_width{0.0};
  int long_weld_rise_percentage{0};  // 0-100%
};

struct JointDeltas {
  double delta_groove_ang{0.0};
  double delta_chamfer_ang{0.0};
  double delta_chamfer_len{0.0};
  double delta_root_face{0.0};
};

struct JointDeviation {
  double slice_angle{0.0};  // Interval [0, 2*PI)
  JointDeltas deltas_left;
  JointDeltas deltas_right;
  double delta_root_gap{0.0};
  double center_line_offset{0.0};

  auto operator<(const JointDeviation &other) const -> bool { return slice_angle < other.slice_angle; }
};

// struct MacsConfig {
//   // x_offset is currently constantly zero. MACS defined with origin in plane parallell to ROCS yx-plane.
//   // Plane containing MACS origin located at left edge of joint (at calibration time)
//   // In simulation: ROCS x coord is in centerline of weld (middle of rootgap). So x-offset of MACS will is distance
//   from
//   // centerline to left edge. When weld object calibration is performed, MACS translation from ROCS is set (y,z) and
//   TCS
//   // and MACS are identical (slides are zeroed)
//   double x;  // Offset from ROCS (negative). Only used in simulation.
//   double y;  // Offset from ROCS (negative)
//   double z;  // Offset from ROCS (negative)
// };

// Translation and orientation of LPCS relative to TCS
struct LpcsConfig {
  double alpha;  // Tilt between laser plane and vertical slide.
  double x;      // Offset from TCS
  double y;      // Offset from TCS
  double z;      // Offset from TCS
};

// Translation of object positioner from MACS
struct OpcsConfig {
  double x;
  double y;
  double z;
};

class SimConfig {
 public:
  SimConfig();
  double target_stickout{};
  int nbr_abw_points{};
  double travel_speed{};
  double root_gap{};
  double total_width{};                 // Total width of simulated work piece
  double joint_bottom_curv_radius{};    // Curvature of the milled joint bottom
  int joint_depth_percentage{};         // The depth of the joint as a percentage of base metal thickness
  int nbr_joint_bottom_points{};        // The number of points used to represent the joint bottom (not ABW points)
  int nbr_slices_per_rev{};             // The number of steps used per revolution to extrude the 3D work piece.
  double drift_speed{};                 // m/rad
  bool ignore_collisions{true};         // To control if torch movements are limited by weld object collisions
  JointDef joint_def_left;              // Definition of the left side of the joint, as seen by scanner
  JointDef joint_def_right;             // Definition of the right side of the joint, as seen by scanner
  std::set<JointDeviation> deviations;  // Deltas for basic joint parameters defined at specific slice angles
  LpcsConfig lpcs_config;               // Definition of LPCS relative to TCS
  // MacsConfig macs_config;             // Definition of ROCS (Rotating Object Coordinate System) relative to MACS
  OpcsConfig opcs_config;  // Definition of OPCS (Object Positioner Coordinate System) relative to MACS
};

}  // namespace deposition_simulator
