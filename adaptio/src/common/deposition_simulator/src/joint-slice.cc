
#include "joint-slice.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <Eigen/Eigen>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <vector>

#include "../sim-config.h"
#include "arc2d.h"
#include "bead.h"
#include "circle2d.h"
#include "cwo.h"
#include "intersection-point.h"
#include "line2d.h"
#include "point2d.h"
#include "point3d.h"
#include "polygon.h"
#include "slice-line.h"

namespace deposition_simulator {

using Eigen::Vector2d;

const double PI                   = std::numbers::pi;
const double GROOVE_FACE_OFFSET   = 3e-3;
const int NBR_ARC_POINTS          = 10;
const double OPT_PRECISION        = 0.01;
const double OPT_DEFAULT_STEP     = 0.0005;
const double BEAD_AREA_FACTOR     = 10.0;
const int OPT_MAX_ITER            = 1000;
const int TOP_SURFACE_LEFT_INDEX  = 0;
const int CHAMFER_LEFT_INDEX      = 1;
const int GROOVE_LEFT_INDEX       = 2;
const int ROOT_FACE_LEFT_INDEX    = 3;
const int ROOT_FACE_RIGHT_INDEX   = 4;
const int GROOVE_RIGHT_INDEX      = 5;
const int CHAMFER_RIGHT_INDEX     = 6;
const int TOP_SURFACE_RIGHT_INDEX = 7;

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
JointSlice::JointSlice(JointDef joint_def_left, JointDef joint_def_right, double slice_angle_rocs, double root_gap,
                       double total_width, double center_line_offset)
    : total_width_(total_width),
      root_gap_(root_gap),
      slice_angle_rocs_(slice_angle_rocs),
      left_edge_to_center_dist_(0.0),
      center_line_offset_(center_line_offset),
      joint_def_left_(joint_def_left),
      joint_def_right_(joint_def_right) {
  ComputeBaseMetalLines();
  ApplyMisAlignment();
}

JointSlice::JointSlice()
    : total_width_(1.0),
      root_gap_(0.0),
      slice_angle_rocs_(0.0),
      left_edge_to_center_dist_(0.0),
      center_line_offset_(0.0),
      joint_def_left_(),
      joint_def_right_() {}

JointSlice::~JointSlice() = default;

auto JointSlice::GetSliceAngle() const -> double { return this->slice_angle_rocs_; }

auto JointSlice::ApplyMisAlignment() -> void {
  // Offset right and left object halves and "project" slice lines onto
  // radial direction of original/common centre axis.
  Circle2d circle;
  Line2d new_radial_line;
  Vector2d dir{};
  Point2d new_start;
  Point2d new_end;
  std::vector<IntersectionPoint> intersections;
  double radial_offset     = 0.0;
  double projection_radius = NAN;
  const double slice_angle = slice_angle_rocs_ + (PI / 2);

  // Loop over all slice lines and calculate new start and end points
  for (auto &slice_line : this->slice_lines_) {
    // if ((slice_line.GetJointPart() & LEFT) == slice_line.GetJointPart()){
    //   radial_offset = this->joint_def_left_.radial_offset;
    // }
    // else if ((slice_line.GetJointPart() & RIGHT) == slice_line.GetJointPart()){
    //   radial_offset = this->joint_def_right_.radial_offset;
    // }
    // else{
    //   continue;
    // }

    if (slice_line.GetJointPart() == TOP_SURFACE_LEFT || slice_line.GetJointPart() == CHAMFER_LEFT ||
        slice_line.GetJointPart() == GROOVE_LEFT || slice_line.GetJointPart() == ROOT_FACE_LEFT) {
      radial_offset = this->joint_def_left_.radial_offset;
    } else if (slice_line.GetJointPart() == TOP_SURFACE_RIGHT || slice_line.GetJointPart() == CHAMFER_RIGHT ||
               slice_line.GetJointPart() == GROOVE_RIGHT || slice_line.GetJointPart() == ROOT_FACE_RIGHT) {
      radial_offset = this->joint_def_right_.radial_offset;
    } else {
      throw std::runtime_error("Mis-alignment applied to non-base metal slice lines is not supported.");
    }

    // Project start point onto new radial section
    projection_radius = slice_line.GetStart().GetY();
    circle            = {0.0, radial_offset, projection_radius};
    dir               = {std::cos(slice_angle), std::sin(slice_angle)};
    new_radial_line   = Line2d(Point2d(0, 0), dir, 1.0);
    intersections     = new_radial_line.Intersect(circle, true, false);

    if (intersections.empty()) {
      throw std::runtime_error("Could not compute new geometry from misalignment.");
    }

    new_start = {slice_line.GetStart().GetX(),
                 intersections[0].ToVector().norm()};  // Should always be first (and only intersection)

    // Project end point onto new radial section
    projection_radius = slice_line.GetEnd().GetY();
    circle            = {0.0, radial_offset, projection_radius};
    dir               = {std::cos(slice_angle), std::sin(slice_angle)};
    new_radial_line   = Line2d(Point2d(0, 0), dir, 1.0);
    intersections     = new_radial_line.Intersect(circle, true, false);

    if (intersections.empty()) {
      throw std::runtime_error("Could not compute new geometry from misalignment.");
    }

    new_end = {slice_line.GetEnd().GetX(),
               intersections[0].ToVector().norm()};  // Should always be first (and only intersection)

    slice_line.SetStart(new_start);
    slice_line.SetEnd(new_end);

    // Also modify the original base metal lines
    if (slice_line.GetJointPart() == TOP_SURFACE_LEFT) {
      base_metal_lines_[TOP_SURFACE_LEFT_INDEX].SetStart(new_start);
      base_metal_lines_[TOP_SURFACE_LEFT_INDEX].SetEnd(new_end);
    } else if (slice_line.GetJointPart() == CHAMFER_LEFT) {
      base_metal_lines_[CHAMFER_LEFT_INDEX].SetStart(new_start);
      base_metal_lines_[CHAMFER_LEFT_INDEX].SetEnd(new_end);
    } else if (slice_line.GetJointPart() == GROOVE_LEFT) {
      base_metal_lines_[GROOVE_LEFT_INDEX].SetStart(new_start);
      base_metal_lines_[GROOVE_LEFT_INDEX].SetEnd(new_end);
    } else if (slice_line.GetJointPart() == ROOT_FACE_LEFT) {
      base_metal_lines_[ROOT_FACE_LEFT_INDEX].SetStart(new_start);
      base_metal_lines_[ROOT_FACE_LEFT_INDEX].SetEnd(new_end);
    } else if (slice_line.GetJointPart() == TOP_SURFACE_RIGHT) {
      base_metal_lines_[TOP_SURFACE_RIGHT_INDEX].SetStart(new_start);
      base_metal_lines_[TOP_SURFACE_RIGHT_INDEX].SetEnd(new_end);
    } else if (slice_line.GetJointPart() == CHAMFER_RIGHT) {
      base_metal_lines_[CHAMFER_RIGHT_INDEX].SetStart(new_start);
      base_metal_lines_[CHAMFER_RIGHT_INDEX].SetEnd(new_end);
    } else if (slice_line.GetJointPart() == GROOVE_RIGHT) {
      base_metal_lines_[GROOVE_RIGHT_INDEX].SetStart(new_start);
      base_metal_lines_[GROOVE_RIGHT_INDEX].SetEnd(new_end);
    } else if (slice_line.GetJointPart() == ROOT_FACE_RIGHT) {
      base_metal_lines_[ROOT_FACE_RIGHT_INDEX].SetStart(new_start);
      base_metal_lines_[ROOT_FACE_RIGHT_INDEX].SetEnd(new_end);
    }
  }
}

// Compute how much the longitudinal weld adds to the radius at this slice angle.
auto JointSlice::ComputeLongWeldAddition(double slice_angle, const JointDef &joint_def) -> double {
  if (std::isnan(joint_def.long_weld_clock_angle)) {
    return 0.0;
  }

  const double pipe_radius          = joint_def.outer_diameter / 2;
  const double long_weld_center_ang = joint_def.long_weld_clock_angle;
  const double ang_width_tot        = joint_def.long_weld_width / pipe_radius;
  const double ang_width_rise       = 0.01 * joint_def.long_weld_rise_percentage * ang_width_tot / 2;
  const double ang_width_fall       = 0.01 * joint_def.long_weld_rise_percentage * ang_width_tot / 2;
  const double start_angle          = long_weld_center_ang - (ang_width_tot / 2);
  const double end_angle            = long_weld_center_ang + (ang_width_tot / 2);
  const double rise_end_angle       = start_angle + ang_width_rise;
  const double fall_start_angle     = end_angle - ang_width_fall;
  const double delta_x_rise         = pipe_radius * ang_width_rise;
  const double delta_x_fall         = pipe_radius * ang_width_fall;
  const double slope_rise           = joint_def.long_weld_height / delta_x_rise;
  const double slope_fall           = -joint_def.long_weld_height / delta_x_fall;

  const Eigen::Vector2d slice_vec      = {-std::sin(slice_angle), std::cos(slice_angle)};
  const Eigen::Vector2d start_vec      = {-std::sin(start_angle), std::cos(start_angle)};
  const Eigen::Vector2d end_vec        = {-std::sin(end_angle), std::cos(end_angle)};
  const Eigen::Vector2d rise_start_vec = {-std::sin(rise_end_angle), std::cos(rise_end_angle)};
  const Eigen::Vector2d fall_start_vec = {-std::sin(fall_start_angle), std::cos(fall_start_angle)};
  const Eigen::Vector2d center_vec     = {-std::sin(long_weld_center_ang), std::cos(long_weld_center_ang)};

  const double cos_to_center = center_vec.dot(slice_vec);
  // Don't check points unreasonly far from long weld.
  if (cos_to_center < 0.0) {
    return 0.0;
  }

  double radial_addition = 0.0;

  // Check if in rise zone
  double cos_to_start = start_vec.dot(slice_vec);
  double cos_to_end   = rise_start_vec.dot(slice_vec);
  double cos_width    = start_vec.dot(rise_start_vec);

  if (cos_width < cos_to_start && cos_width <= cos_to_end && cos_to_start > 0 && cos_to_end > 0) {
    radial_addition = slope_rise * std::acos(cos_to_start) * pipe_radius;
    return radial_addition;
  }

  // Check if in flat zone
  cos_to_start = rise_start_vec.dot(slice_vec);
  cos_to_end   = fall_start_vec.dot(slice_vec);
  cos_width    = rise_start_vec.dot(fall_start_vec);

  if (cos_width < cos_to_start && cos_width <= cos_to_end && cos_to_start > 0 && cos_to_end > 0) {
    radial_addition = joint_def.long_weld_height;
    return radial_addition;
  }

  // Check if in fall zone
  cos_to_start = fall_start_vec.dot(slice_vec);
  cos_to_end   = end_vec.dot(slice_vec);
  cos_width    = fall_start_vec.dot(end_vec);

  if (cos_width < cos_to_start && cos_width <= cos_to_end && cos_to_start > 0 && cos_to_end > 0) {
    radial_addition = joint_def.long_weld_height + slope_fall * std::acos(cos_to_start) * pipe_radius;
    return radial_addition;
  }

  return radial_addition;
}

auto JointSlice::ComputeBaseMetalLines() -> void {
  // This could be made less verbose and use better variable names. Ref. to docs/figs for meaning of variables.
  // Just copying code from Scilab imlementation for now...
  double d1          = joint_def_left_.basemetal_thickness;
  double d2          = joint_def_right_.basemetal_thickness;
  double r1          = joint_def_left_.outer_diameter / 2;
  double r2          = joint_def_right_.outer_diameter / 2;
  const double width = total_width_;

  // Add longitudinal weld if located at this slice angle.
  double long_weld_addition  = NAN;
  long_weld_addition         = ComputeLongWeldAddition(slice_angle_rocs_, this->joint_def_left_);
  d1                        += long_weld_addition;
  r1                        += long_weld_addition;
  long_weld_addition         = ComputeLongWeldAddition(slice_angle_rocs_, this->joint_def_right_);
  d2                        += long_weld_addition;
  r2                        += long_weld_addition;

  // Left side
  std::array<SliceLine, 4> left_lines;
  double ga = joint_def_left_.groove_ang;   // Angle
  double ca = joint_def_left_.chamfer_ang;  // Angle
  double ka = joint_def_left_.root_face;    // Weight
  double kc = joint_def_left_.chamfer_len;  // Weight

  // Compute joint vectors, left.
  Vector2d ss = Vector2d((root_gap_ / 2) + center_line_offset_, 0);  // y = 0 by definition of slice origin
  Vector2d aa = Vector2d(0, 1);
  Vector2d bb = Vector2d(std::sin(ga), std::cos(ga));
  Vector2d cc = Vector2d(std::cos(ca), std::sin(ca));
  Vector2d dd = Vector2d(1, 0);

  double kb = (d1 - ka * aa(1) - kc * cc(1) - ss(1)) / bb(1);
  // double kd = (-w/2 - kb*b(0) - kc*c(0) - s(0)) / d(0);
  double kd = (width / 2 - kb * bb(0) - kc * cc(0) - ss(0)) / dd(0);

  // Create SliceLines from vectors and weights. Each line start where the preceeding one ends.
  Point2d start(ss(0), ss(1));
  left_lines[0] = SliceLine(start, aa, ka);                   // Root face
  left_lines[1] = SliceLine(left_lines[0].GetEnd(), bb, kb);  // Groove face
  left_lines[2] = SliceLine(left_lines[1].GetEnd(), cc, kc);  // Chamfer
  left_lines[3] = SliceLine(left_lines[2].GetEnd(), dd, kd);  // Top surface

  // Right side
  std::array<SliceLine, 4> right_lines;
  ga = joint_def_right_.groove_ang;   // Angle
  ca = joint_def_right_.chamfer_ang;  // Angle
  ka = joint_def_right_.root_face;    // Weight
  kc = joint_def_right_.chamfer_len;  // Weight

  // Compute joint vectors, right.
  ss = Vector2d((-root_gap_ / 2) + center_line_offset_, d1 + (r2 - r1) - d2);  // y = 0 by definition of slice origin
  aa = Vector2d(0, 1);
  bb = Vector2d(-std::sin(ga), std::cos(ga));
  cc = Vector2d(-std::cos(ca), std::sin(ca));
  dd = Vector2d(-1, 0);

  kb = (d1 + r2 - r1 - ka * aa(1) - kc * cc(1) - ss(1)) / bb(1);
  // kd = (w/2 - kb*b(0) - kc*c(0) - s(0)) /d(0);
  kd = (-width / 2 - kb * bb(0) - kc * cc(0) - ss(0)) / dd(0);

  // Create SliceLines from vectors and weights. Each line start where the preceeding one ends.
  start          = Point2d(ss(0), ss(1));
  right_lines[0] = SliceLine(start, aa, ka);
  right_lines[1] = SliceLine(right_lines[0].GetEnd(), bb, kb);
  right_lines[2] = SliceLine(right_lines[1].GetEnd(), cc, kc);
  right_lines[3] = SliceLine(right_lines[2].GetEnd(), dd, kd);

  // Join left and right side lines, reverse left so that we get all lines in left to right order and direction.
  left_lines[0].Reverse();
  left_lines[1].Reverse();
  left_lines[2].Reverse();
  left_lines[3].Reverse();

  // Find left groove edge to use as x reference
  // std::unique_ptr<Point2d> left_edge = left_lines[1].Intersect(left_lines[3], false, false);
  std::unique_ptr<Point2d> left_edge = left_lines[1].Intersect(left_lines[3], false, false);

  if (left_edge == nullptr) {
    throw std::runtime_error("Could not determine left groove edge.");
  }

  // To express lines w.r.t ROCS in xy plane.
  // The origin used in calculations is x = 0 <==> joint center line, y = 0 <==> left inside of base metal.
  // Define translation to get it relative to the weld object symmmetry axis (ROCS x)
  // x = 0 should be given by left joint edge (top/groove interpolated intersection)
  // y = 0 should be r = 0 in ROCS cylindrical coords.

  Vector2d shift                  = {-left_edge->GetX() + center_line_offset_,
                                     (joint_def_left_.outer_diameter / 2) - joint_def_left_.basemetal_thickness};
  this->left_edge_to_center_dist_ = shift(0);

  base_metal_lines_[TOP_SURFACE_LEFT_INDEX]  = left_lines[3];
  base_metal_lines_[CHAMFER_LEFT_INDEX]      = left_lines[2];
  base_metal_lines_[GROOVE_LEFT_INDEX]       = left_lines[1];
  base_metal_lines_[ROOT_FACE_LEFT_INDEX]    = left_lines[0];
  base_metal_lines_[ROOT_FACE_RIGHT_INDEX]   = right_lines[0];
  base_metal_lines_[GROOVE_RIGHT_INDEX]      = right_lines[1];
  base_metal_lines_[CHAMFER_RIGHT_INDEX]     = right_lines[2];
  base_metal_lines_[TOP_SURFACE_RIGHT_INDEX] = right_lines[3];

  base_metal_lines_[TOP_SURFACE_LEFT_INDEX].SetJointPart(TOP_SURFACE_LEFT);
  base_metal_lines_[CHAMFER_LEFT_INDEX].SetJointPart(CHAMFER_LEFT);
  base_metal_lines_[GROOVE_LEFT_INDEX].SetJointPart(GROOVE_LEFT);
  base_metal_lines_[ROOT_FACE_LEFT_INDEX].SetJointPart(ROOT_FACE_LEFT);
  base_metal_lines_[ROOT_FACE_RIGHT_INDEX].SetJointPart(ROOT_FACE_RIGHT);
  base_metal_lines_[GROOVE_RIGHT_INDEX].SetJointPart(GROOVE_RIGHT);
  base_metal_lines_[CHAMFER_RIGHT_INDEX].SetJointPart(CHAMFER_RIGHT);
  base_metal_lines_[TOP_SURFACE_RIGHT_INDEX].SetJointPart(TOP_SURFACE_RIGHT);

  slice_lines_.clear();

  for (auto &line : base_metal_lines_) {
    line.ShiftLine(shift);
    slice_lines_.push_back(line);
  }
}

auto JointSlice::ComputeMergePoints(const Circle2d &circle) -> std::vector<IntersectionPoint> {
  std::vector<IntersectionPoint> final_intersections{};
  std::array<IntersectionPoint, 2> intersections = {};
  int nbr_intersections                          = 0;

  // Check intersection with existing slice lines.
  for (auto &line : this->slice_lines_) {
    nbr_intersections = line.Intersect(circle, intersections);

    if (nbr_intersections == 0) {
      continue;
    }

    final_intersections.push_back(intersections[0]);

    if (nbr_intersections > 1) {
      final_intersections.push_back(intersections[1]);
    }
  }

  return final_intersections;
}

auto JointSlice::CreateBead(Circle2d &circle, bool concave) -> std::unique_ptr<Bead> {
  std::vector<IntersectionPoint> final_intersections =
      ComputeMergePoints(circle);  // All intersection between the circle and all lines.

  if (final_intersections.size() < 2) {
    return nullptr;
  }

  // if (finalIntersections.size() > 2)
  //   throw std::range_error("Mathematically inconsistent intersection results.");

  // Sort and take the two top points.
  if (concave) {
    std::ranges::sort(final_intersections.begin(), final_intersections.end(),
                      [circle](IntersectionPoint &p1, IntersectionPoint &p2) {
                        return (p1.GetPolarAngle(circle.GetCenter(), true) - 3 * PI / 2 >
                                p2.GetPolarAngle(circle.GetCenter(), true) - 3 * PI / 2);
                      });
  } else {
    std::ranges::sort(final_intersections.begin(), final_intersections.end(),
                      [circle](IntersectionPoint &p1, IntersectionPoint &p2) {
                        return (p1.GetPolarAngle(circle.GetCenter(), true) - PI / 2 >
                                p2.GetPolarAngle(circle.GetCenter(), true) - PI / 2);
                      });
  }

  const double ang0 = final_intersections[0].GetPolarAngle(circle.GetCenter(), false);  //[-PI,PI]
  const double ang1 = final_intersections[1].GetPolarAngle(circle.GetCenter(), false);  //[-PI,PI]

  // Start of arc must always be the point closest to PI
  // with new def of left <==> +x and right <==> -x start of arc must always be point closest to zero!!!
  double start_angle             = ang1;
  double end_angle               = ang0;
  IntersectionPoint merge_point1 = final_intersections[1];
  IntersectionPoint merge_point2 = final_intersections[0];

  if ((abs(start_angle) > abs(end_angle))) {
    start_angle  = ang0;
    end_angle    = ang1;
    merge_point1 = final_intersections[0];
    merge_point2 = final_intersections[1];
  }

  // Change range [-PI, PI] --> [0, 2PI]
  start_angle = (start_angle < 0) ? (2 * std::numbers::pi) + start_angle : start_angle;
  end_angle   = (end_angle < 0) ? (2 * std::numbers::pi) + end_angle : end_angle;
  // std::cout << "Arc angles: " << start_angle << ", " << end_angle << "\n";

  // Stuff to return
  const Arc2d arc = Arc2d(circle, start_angle, end_angle, concave);
  return std::make_unique<Bead>(Bead(arc, merge_point1, merge_point2));
}

auto JointSlice::ComputeNewSurface(Bead &bead, int nbr_arc_points) -> std::vector<SliceLine> {
  SliceLine bead_line;
  const std::vector<Line2d> arc_lines = bead.GetArc().Discretize(nbr_arc_points);
  std::vector<SliceLine> new_lines;
  // std::vector<SliceLine> beadLines();

  bool found1st = false;
  bool found2nd = false;
  // int nbr_new   = 0;

  for (SliceLine &curr_line : this->slice_lines_) {
    if (&curr_line == bead.GetStart().GetIntersectedLine())  // Found first merge line
    {
      // Modify merge line to end at merge point
      curr_line.SetEnd({bead.GetStart().GetX(), bead.GetStart().GetY()});
      new_lines.push_back(curr_line);
      found1st = true;

      for (const Line2d &it_line : arc_lines)  // Insert new bead lines
      {
        bead_line = SliceLine::FromPoints(it_line.GetStart(), it_line.GetEnd());
        bead_line.SetJointPart(WELD_METAL);
        new_lines.push_back(bead_line);
        // nbr_new++;
      }
    }

    if (&curr_line == bead.GetEnd().GetIntersectedLine())  // Found second merge line
    {
      // Modify merge line to start at merge point
      curr_line.SetStart({bead.GetEnd().GetX(), bead.GetEnd().GetY()});
      new_lines.push_back(curr_line);
      found2nd = true;
      continue;
    }

    if (found1st == found2nd)  // Outside mergepoints
    {
      new_lines.push_back(curr_line);  // Just keep existing line.
    }
  }

  return new_lines;
}

auto JointSlice::ComputeBeadPolygon(Bead &bead) -> Polygon {
  SliceLine bead_line;
  const std::vector<Line2d> arc_lines = bead.GetArc().Discretize(NBR_ARC_POINTS);
  std::vector<SliceLine> bead_lines;

  bool found1st = false;
  bool found2nd = false;
  int nbr_new   = 0;

  for (SliceLine &curr_line : this->slice_lines_) {
    if (&curr_line == bead.GetStart().GetIntersectedLine()) {
      for (Line2d const &it_line : arc_lines)  // Insert new bead lines
      {
        bead_line = SliceLine::FromPoints(it_line.GetStart(), it_line.GetEnd());
        bead_lines.push_back(bead_line);
        nbr_new++;
      }

      // Modify merge line to end at merge point
      bead_line = SliceLine::FromPoints(curr_line.GetEnd(), bead.GetStart());
      // beadLines.insert(beadLines.begin() + nbrNew, beadLine);
      bead_lines.push_back(bead_line);
      found1st = true;
      continue;
    }

    if (&curr_line == bead.GetEnd().GetIntersectedLine()) {
      bead_line = SliceLine::FromPoints(bead.GetEnd(), curr_line.GetStart());
      // beadLines.push_back( beadLine);
      bead_lines.insert(bead_lines.begin() + nbr_new, bead_line);
      found2nd = true;
    }

    if (found1st && !found2nd)  // Between merge points
    {
      bead_line = SliceLine::FromPoints(curr_line.GetStart(), curr_line.GetEnd());
      bead_line.Reverse();
      bead_lines.insert(bead_lines.begin() + nbr_new, bead_line);
    }

    if (found2nd) {
      break;
    }
  }

  Polygon polygon;
  for (auto const &edge : bead_lines) {
    // std::cout << edge.GetStart().GetX() << "," << edge.GetStart().GetY() << "," << edge.GetEnd().GetX() << "," <<
    // edge.GetEnd().GetY() << std::endl;
    polygon.AddEdge(Line2d::FromPoints(edge.GetStart(), edge.GetEnd()));
  }
  return polygon;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
auto JointSlice::AddJointBottom(double joint_depth, double curv_radius, int nbr_arc_points) -> void {
  // double y_max = base_metal_lines_[0].GetStart().GetY();

  // if (base_metal_lines_[NBR_BASE_METAL_LINES_PER_SIDE * 2 - 1].GetStart().GetY() > y_max) {
  //   y_max = base_metal_lines_[NBR_BASE_METAL_LINES_PER_SIDE * 2 - 1].GetStart().GetY();
  // }
  const double y_max = this->FindHighestPointInJoint();

  Circle2d circle =
      Circle2d(center_line_offset_ + left_edge_to_center_dist_, y_max - joint_depth + curv_radius, curv_radius);
  std::unique_ptr<Bead> bead = CreateBead(circle, true);

  if (bead == nullptr) {
    throw std::runtime_error("Joint bottom not intersecting base metal.");
  }

  const std::vector<SliceLine> new_lines = ComputeNewSurface(*bead, nbr_arc_points);

  this->slice_lines_.clear();

  for (const auto &sl : new_lines) {
    this->slice_lines_.push_back(sl);
  }
}

auto JointSlice::FindLowestPointInJoint() -> double {
  double y_min = this->slice_lines_[0].GetMinY();

  for (auto const &line : this->slice_lines_) {
    // if (y_min > line.GetMinY()) {
    y_min = std::min(line.GetMinY(), y_min);
    //}
  }

  return y_min;
}

auto JointSlice::FindHighestPointInJoint() -> double {
  double y_max = this->slice_lines_[0].GetMaxY();

  for (auto const &line : this->slice_lines_) {
    // if (y_max < line.GetMaxY()) {
    y_max = std::max(line.GetMaxY(), y_max);
    //}
  }

  return y_max;
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity, bugprone-easily-swappable-parameters)
auto JointSlice::AddBead(double target_bead_area, double bead_radius, double target_stickout, const Point2d &torchpos,
                         bool use_process_dependent_bead) -> void {
  // std::cout << "Adding bead to slice\n";

  const double y_min = FindLowestPointInJoint();

  if (use_process_dependent_bead) {
    // Find actual stickout and adjust the target bead area
    Eigen::Vector2d wire_dir = {0.0, -1.0};
    const Line2d wire_line{torchpos, wire_dir};
    std::optional<Point2d> surf_point = FindIntersectionWithSurface(wire_line);
    const double y_surf               = surf_point->GetY();
    const double actual_stickout      = std::abs(torchpos.GetY() - y_surf);
    const double stickout_fraction    = actual_stickout / target_stickout;
    if (stickout_fraction < 1.0) {
      target_bead_area = target_bead_area * (std::exp(BEAD_AREA_FACTOR * stickout_fraction) - 1.0) /
                         (std::exp(BEAD_AREA_FACTOR) - 1.0);
    }
  }

  Circle2d circle;
  std::unique_ptr<Bead> bead = nullptr;  // = CreateBead(circle, false);
  Polygon poly_bead;
  double area           = NAN;
  double step_size      = OPT_DEFAULT_STEP;
  bool root_bracketed   = false;
  double area_diff      = INFINITY;
  double prev_area_diff = NAN;
  const double xc       = torchpos.GetX();
  double ya             = y_min - bead_radius;  // Lowest point in interval. ya < yb
  double yb             = ya;                   // Highest point in interval
  double yc             = 0.0;                  // Center point of interval
  int iter_count        = 0;

  // std::ofstream log_file;
  // log_file.open("bead_opt.txt");
  //  Interval bisection until precision reached
  while (std::abs(area_diff) > OPT_PRECISION * target_bead_area) {
    // log_file << "iter_count: " << iter_count << "\n";
    if (iter_count == OPT_MAX_ITER) {
      // for (auto &edge : polyBead.GetEdgePoints()) std::cout << edge.GetX() << "," << edge.GetY() << std::endl;
      // log_file.close();
      throw std::runtime_error("Reached max iterations before target bead area.");
    }
    yc     = (ya + yb) / 2;
    circle = Circle2d(xc, yc, bead_radius);
    bead   = CreateBead(circle, false);

    if (bead == nullptr)  // Bead circle does not intersect existing joint profile.
    {
      yb += step_size;
      iter_count++;
      // log_file << "Action: -1 \n";
      continue;
    }

    poly_bead = ComputeBeadPolygon(*bead);
    area      = poly_bead.Area();
    area_diff = area - target_bead_area;
    // log_file << "bead_radius:" << bead_radius << "\n";
    // log_file << "Area:" << area << "\n";
    // log_file << "Target:" << target_bead_area << "\n";
    // log_file << "Diff:" << area_diff << "\n";
    // log_file << "y_center:" << yc << "\n";
    // log_file << "x_center:" << xc << "\n";
    // log_file << "bead_start:" << bead->GetStart().GetX() << "," << bead->GetStart().GetY() << "\n";
    // log_file << "bead_end:" << bead->GetEnd().GetX() << "," << bead->GetEnd().GetY() << "\n";
    // log_file << "bead_vertices: ";
    // for (auto &pb : poly_bead.GetEdgePoints()) {
    //   log_file << pb.GetX() << "," << pb.GetY() << ",";
    // }
    // log_file << "\n";

    if (!std::isnan(prev_area_diff)) {
      if (!root_bracketed) {
        if (std::signbit(area_diff) != std::signbit(prev_area_diff)) {
          // log_file << "Action: 1\n";
          root_bracketed = true;
          yb             = yc;
        } else {
          // log_file << "Action: 2\n";
          yb += step_size;
        }
      } else {
        if (std::signbit(area_diff) != std::signbit(prev_area_diff)) {
          // log_file << "Action: 3\n";
          yb = yc;
        } else {
          // if (std::abs(prev_area_diff) < std::abs(area_diff)) {
          // Area diff not shrinking. Restart procedure at this point
          // throw std::runtime_error("Bead optimizer not converging. Torch outside joint?");
          // log_file << "Action: 4a\n";
          // ya = yc;
          // yb = yc;
          // root_bracketed = false;
          // prev_area_diff = NAN;
          // }
          // else {
          // log_file << "Action: 4b\n";
          ya             = yc;
          prev_area_diff = area_diff;
          // }
        }
      }
    } else {
      if (area_diff > 0)  // First iteration must have a too small bead, otherwise stepsize is to big.
      {
        // log_file << "Action: 5\n";
        yb        -= step_size;      // One step back before recalculating with smaller step.
        step_size  = step_size / 2;  // Reduce step and continue to do the same area calculation again.
        continue;
      }
      // log_file << "Action: 6\n";
      prev_area_diff  = area_diff;
      yb             += step_size;
    }

    iter_count++;
  }
  // log_file.close();
  // Merge the new bead with the existing joint + weld metal
  const std::vector<SliceLine> new_lines = ComputeNewSurface(*bead, NBR_ARC_POINTS);
  this->slice_lines_.clear();
  for (const auto &sl : new_lines) {
    this->slice_lines_.push_back(sl);
  }
}

auto JointSlice::GetAbwPoints(bool allow_cap_points) const -> std::unique_ptr<std::vector<std::optional<Point2d>>> {
  std::vector<std::optional<Point2d>> abw;
  abw.reserve(DEFAULT_NBR_ABW_POINTS);

  // First try determine the two leftest and rightest points. May not be possible if top edges
  // are covered by beads. In such cases, fall back to original base metal lines and find
  // top surface at the same horizontal position as original groove edge.
  const auto abw0 = ComputeTopLeftAbw();
  const auto abw6 = ComputeTopRightAbw();

  if (!abw0 || !abw6) {
    return std::make_unique<std::vector<std::optional<Point2d>>>(std::move(abw));
  }

  const auto abw1                = ComputeBottomLeftAbw(*abw0);
  const auto abw5                = ComputeBottomRightAbw(*abw6);
  const bool both_sides_detected = abw1 && abw5;

  abw.push_back(abw0);
  abw.push_back(abw1);
  abw.push_back(abw5);
  abw.push_back(abw6);

  if (!both_sides_detected) {
    return std::make_unique<std::vector<std::optional<Point2d>>>(std::move(abw));
  }

  // Compute intermediate bottom points
  constexpr int FIXED_POINTS = 4;
  const int n_bottom         = DEFAULT_NBR_ABW_POINTS - FIXED_POINTS;
  const double x_left        = abw5->GetX();
  const double x_right       = abw1->GetX();
  const double x_step        = (x_right - x_left) / (n_bottom + 1);
  const double y0            = GetMaxY();
  const Eigen::Vector2d dir(0.0, 1.0);

  std::vector<std::optional<Point2d>> bottom_points;
  bottom_points.reserve(n_bottom);

  for (int i = 1; i <= n_bottom; ++i) {
    const Line2d tmp_line(Point2d(x_right - i * x_step, y0), dir);
    bottom_points.push_back(FindIntersectionWithSurface(tmp_line));
  }

  // Insert bottom points between bottom-left and bottom-right
  abw.insert(abw.begin() + 2, bottom_points.begin(), bottom_points.end());

  return std::make_unique<std::vector<std::optional<Point2d>>>(std::move(abw));
}

auto JointSlice::FindIntersectionWithSurface(const Line2d &line) const -> std::optional<Point2d> {
  for (const auto &slice_line : this->slice_lines_) {
    if (auto p_int = line.Intersect(slice_line, false, true)) {
      return *p_int;
    }
  }
  return std::nullopt;
}

auto JointSlice::ComputeBottomLeftAbw(const Point2d &top_left_abw) const -> std::optional<Point2d> {
  // ABW 1
  const Point2d line_start{top_left_abw.GetX() - GROOVE_FACE_OFFSET, top_left_abw.GetY()};
  const Vector2d groove_face_dir = {std::sin(joint_def_left_.groove_ang), std::cos(joint_def_left_.groove_ang)};
  const Line2d intersection_line = Line2d(line_start, groove_face_dir, 1.0);
  auto intersection_point        = FindIntersectionWithSurface(intersection_line);
  Point2d abw1;
  double y_coord = NAN;

  if (!intersection_point.has_value()) {
    return std::nullopt;
  }

  if (intersection_point.value().GetY() > top_left_abw.GetY()) {
    y_coord = top_left_abw.GetY();
  } else {
    y_coord = intersection_point.value().GetY();
  }

  abw1 = {intersection_point.value().GetX() + GROOVE_FACE_OFFSET, y_coord};

  return abw1;
}

auto JointSlice::ComputeBottomRightAbw(const Point2d &top_right_abw) const -> std::optional<Point2d> {
  // ABW 5
  Point2d line_start{top_right_abw.GetX() + GROOVE_FACE_OFFSET, top_right_abw.GetY()};
  Vector2d groove_face_dir = {-std::sin(joint_def_right_.groove_ang), std::cos(joint_def_right_.groove_ang)};
  Line2d intersection_line = Line2d(line_start, groove_face_dir, 1.0);
  auto intersection_point  = FindIntersectionWithSurface(intersection_line);
  Point2d abw5;
  double y_coord = NAN;

  if (!intersection_point.has_value()) {
    return std::nullopt;
  }

  if (intersection_point.value().GetY() > top_right_abw.GetY()) {
    y_coord = top_right_abw.GetY();
  } else {
    y_coord = intersection_point.value().GetY();
  }

  abw5 = {intersection_point.value().GetX() - GROOVE_FACE_OFFSET, y_coord};

  return abw5;
}

auto JointSlice::ComputeTopLeftAbw() const -> std::optional<Point2d> {
  // ABW 0

  std::unique_ptr<Point2d> abw0_orig;
  std::optional<Point2d> abw0_curr;
  const SliceLine *ts_left = &base_metal_lines_[TOP_SURFACE_LEFT_INDEX];
  const SliceLine *gf_left = &base_metal_lines_[GROOVE_LEFT_INDEX];

  // Find intersection point between original groove face and top surface
  abw0_orig = ts_left->Intersect(*gf_left, false, false);

  // Check if there is a surface currently above the original intersection point
  const Line2d intersection_line = {
      *abw0_orig, {0, 1},
       1.0
  };
  abw0_curr = FindIntersectionWithSurface(intersection_line);

  if (abw0_curr.has_value() && abw0_curr->GetY() > abw0_orig->GetY()) {
    return abw0_curr;
  }

  return *abw0_orig;
}

auto JointSlice::ComputeTopRightAbw() const -> std::optional<Point2d> {
  // ABW 6

  std::unique_ptr<Point2d> abw6_orig;
  std::optional<Point2d> abw6_curr;
  const SliceLine *ts_right = &base_metal_lines_[TOP_SURFACE_RIGHT_INDEX];
  const SliceLine *gf_right = &base_metal_lines_[GROOVE_RIGHT_INDEX];

  // Find intersection point between original groove face and top surface
  abw6_orig = ts_right->Intersect(*gf_right, false, false);

  // Check if there is a surface currently above the original intersection point
  Line2d intersection_line = {
      *abw6_orig, {0, 1},
       1.0
  };
  abw6_curr = FindIntersectionWithSurface(intersection_line);

  if (abw6_curr.has_value() && abw6_curr->GetY() > abw6_orig->GetY()) {
    return abw6_curr.value();
  }

  return *abw6_orig;
}

auto JointSlice::GetSlicePoints() const -> std::vector<Point3d> {
  std::vector<Point3d> slice_points;
  double x_coord{NAN};
  double y_coord{NAN};
  double z_coord{NAN};
  double r_coord{NAN};

  for (const auto &line : this->slice_lines_) {
    r_coord = line.GetStart().GetY();
    x_coord = line.GetStart().GetX();
    y_coord = -std::sin(slice_angle_rocs_) * r_coord;
    z_coord = std::cos(slice_angle_rocs_) * r_coord;
    slice_points.emplace_back(x_coord, y_coord, z_coord, ROCS);
  }

  // Add the end of the last slice line.
  r_coord = slice_lines_.back().GetStart().GetY();
  x_coord = slice_lines_.back().GetEnd().GetX();
  y_coord = -std::sin(slice_angle_rocs_) * r_coord;
  z_coord = std::cos(slice_angle_rocs_) * r_coord;
  slice_points.emplace_back(x_coord, y_coord, z_coord, ROCS);

  // Continue to right inner side
  r_coord = slice_lines_.back().GetStart().GetY() - joint_def_right_.basemetal_thickness;
  y_coord = -std::sin(slice_angle_rocs_) * r_coord;
  z_coord = std::cos(slice_angle_rocs_) * r_coord;
  slice_points.emplace_back(x_coord, y_coord, z_coord, ROCS);

  // To left inner side
  x_coord = slice_lines_.front().GetStart().GetX();
  slice_points.emplace_back(x_coord, y_coord, z_coord, ROCS);

  // Back to left point (start)
  r_coord = slice_lines_.front().GetStart().GetY();
  y_coord = -std::sin(slice_angle_rocs_) * r_coord;
  z_coord = std::cos(slice_angle_rocs_) * r_coord;
  slice_points.emplace_back(x_coord, y_coord, z_coord, ROCS);

  return slice_points;
}

auto JointSlice::GetSlicePoints(std::vector<double> &x_coords, std::vector<double> &y_coords) const -> void {
  // const double innerRadiusLeft = joint_def_left_.outer_diameter - joint_def_left_.basemetal_thickness;

  for (const auto &line : this->slice_lines_) {
    x_coords.push_back(line.GetStart().GetX());
    y_coords.push_back(line.GetStart().GetY());
    x_coords.push_back(line.GetEnd().GetX());
    y_coords.push_back(line.GetEnd().GetY());
  }
}

auto JointSlice::GetMinX() const -> double {
  double x_min = INFINITY;

  for (const auto &line : slice_lines_) {
    x_min = std::min(x_min, line.GetMinX());
  }

  return x_min;
}

auto JointSlice::GetMaxX() const -> double {
  double x_max = -std::numeric_limits<double>::infinity();

  for (const auto &line : slice_lines_) {
    x_max = std::max(x_max, line.GetMaxX());
  }

  return x_max;
}

auto JointSlice::GetMaxY() const -> double {
  double y_max = -std::numeric_limits<double>::infinity();

  for (const auto &line : slice_lines_) {
    y_max = std::max(y_max, line.GetMaxY());
  }

  return y_max;
}

}  // namespace deposition_simulator
