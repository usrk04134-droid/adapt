#pragma once

#include <array>
#include <memory>
#include <vector>

#include "../sim-config.h"
#include "bead.h"
#include "circle2d.h"
#include "intersection-point.h"
#include "point2d.h"
#include "point3d.h"
#include "polygon.h"
#include "slice-line.h"

namespace deposition_simulator {

const int NBR_BASE_METAL_LINES_PER_SIDE = 4;

class JointSlice {
 private:
  double total_width_;
  double root_gap_;
  double slice_angle_rocs_;
  double left_edge_to_center_dist_;
  double center_line_offset_;
  std::array<SliceLine, static_cast<std::size_t>(NBR_BASE_METAL_LINES_PER_SIDE * 2)> base_metal_lines_;
  std::vector<SliceLine> slice_lines_;
  JointDef joint_def_left_;
  JointDef joint_def_right_;

  // Private methods
  auto ComputeBaseMetalLines() -> void;
  auto ApplyMisAlignment() -> void;
  auto CreateBead(Circle2d &circle, bool concave) -> std::unique_ptr<Bead>;
  auto ComputeBeadPolygon(Bead &bead) -> Polygon;
  auto ComputeMergePoints(const Circle2d &circle) -> std::vector<IntersectionPoint>;
  auto ComputeNewSurface(Bead &bead, int nbr_arc_points) -> std::vector<SliceLine>;
  auto FindLowestPointInJoint() -> double;
  auto FindHighestPointInJoint() -> double;
  auto ComputeTopLeftAbw() const -> std::optional<Point2d>;
  auto ComputeTopRightAbw() const -> std::optional<Point2d>;
  auto ComputeBottomLeftAbw(const Point2d &top_left_abw) const -> std::optional<Point2d>;
  auto ComputeBottomRightAbw(const Point2d &top_right_abw) const -> std::optional<Point2d>;
  auto FindIntersectionWithSurface(const Line2d &line) const -> std::optional<Point2d>;

 public:
  JointSlice();
  JointSlice(JointDef joint_def_left, JointDef joint_def_right, double slice_angle_rocs, double root_gap,
             double total_width, double center_line_offset);
  ~JointSlice();

  static auto ComputeLongWeldAddition(double slice_angle, const JointDef &joint_def) -> double;
  auto AddJointBottom(double joint_depth, double curv_radius, int nbr_arc_points) -> void;
  auto AddBead(double bead_area, double bead_radius, double target_stickout, const Point2d &torchpos,
               bool use_process_dependent_bead = false) -> void;
  auto GetSlicePoints() const -> std::vector<Point3d>;
  auto GetSlicePoints2d() const -> std::vector<Point2d>;
  auto GetHighResAbwPoints() const -> std::vector<Point2d>;
  auto GetAbwPoints(bool allow_cap_points = false) const -> std::vector<std::optional<Point2d>>;
  auto GetSliceAngle() const -> double;
  auto GetMinX() const -> double;
  auto GetMaxX() const -> double;
  auto GetMaxY() const -> double;
};

}  // namespace deposition_simulator
