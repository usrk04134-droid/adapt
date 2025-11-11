#pragma once

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <numeric>
#include <vector>

#include "calibration/calibration_configuration.h"
#include "common/geometric_primitives/src/line2d.h"
#include "common/groove/point.h"
#include "common/logging/application_log.h"
#include "joint_geometry/joint_geometry.h"

namespace calibration {

using geometric_primitives::Line2d;
using geometric_primitives::Point2d;

const double REDUCTION_FACTOR = 0.95;
const double HALF             = 0.5;

// Note: In this file we reason about the coordinates for the wire tip in
// relation to the groove for simplicity. However, GenerateCalibrationDots will be
// called with common::Point coordinates which are defined as the location of
// the tip of the contact tube. This is fine since it is only the relative
// position values which define the grid.

struct GridLayout {
  double x_center{};  // Center between c_left and c_right
  double z_start{};   // Starting z position (bottom row, at c_z)
  double spacing{};   // Spacing between points
};

struct GridPoint {
  int index_x{};  // Horizontal index (relative to center)
  int index_z{};  // Vertical index (upwards)
  std::shared_ptr<GridLayout> layout;

  auto GetX() const -> double { return layout->x_center + (index_x * layout->spacing); }
  auto GetZ() const -> double { return layout->z_start + (index_z * layout->spacing); }
};

// TODO: Move this function to another file
inline auto ValidateAndCalculateGrooveTopCenter2(const joint_geometry::JointGeometry& joint_geometry,
                                                 double wire_diameter, double stickout,
                                                 const common::Point& left_wall_touch_position,
                                                 const common::Point& right_wall_touch_position,
                                                 const common::Point& top_touch_position)
    -> std::optional<common::Point> {
  // For now, assume symmetric joint so left and right depths are the same
  // TODO(zachjz): Change the following two lines when asymmetric joints are supported.
  const double groove_depth_left  = joint_geometry.groove_depth_mm;
  const double groove_depth_right = joint_geometry.groove_depth_mm;
  const double top_height_diff    = groove_depth_right - groove_depth_left;
  const double wire_radius        = wire_diameter * HALF;

  // Check if top touch point is left or right
  const bool top_touch_is_left_side = top_touch_position.horizontal > left_wall_touch_position.horizontal;
  Point2d top_left_touch_k;
  Point2d top_right_touch_h;

  // Special handling of top touch points based on which side of the joint was touched
  if (top_touch_is_left_side) {
    // Left top --> measured K, computed H
    top_left_touch_k  = {top_touch_position.horizontal, top_touch_position.vertical - stickout};
    top_right_touch_h = {0, top_left_touch_k.GetY() + top_height_diff};
  } else {
    // Right top --> measured H, computed K
    top_right_touch_h = {top_touch_position.horizontal, top_touch_position.vertical - stickout};
    top_left_touch_k  = {0, top_right_touch_h.GetY() - top_height_diff};
  }

  // Additional measured surface points (from touch sense)
  const Point2d left_wall_touch_b  = {left_wall_touch_position.horizontal + wire_radius,
                                      left_wall_touch_position.vertical - stickout};
  const Point2d right_wall_touch_d = {right_wall_touch_position.horizontal - wire_radius,
                                      right_wall_touch_position.vertical - stickout};

  // Computed surface points. Change these if additional touch points are added to the touch sense procedure.
  const Point2d top_left_touch_l   = {top_left_touch_k.GetX() - 1.0, top_left_touch_k.GetY()};
  const Point2d left_wall_touch_j  = {std::sin(joint_geometry.left_joint_angle_rad) + left_wall_touch_b.GetX(),
                                      std::cos(joint_geometry.left_joint_angle_rad) + left_wall_touch_b.GetY()};
  const Point2d right_wall_touch_i = {-std::sin(joint_geometry.right_joint_angle_rad) + right_wall_touch_d.GetX(),
                                      std::cos(joint_geometry.right_joint_angle_rad) + right_wall_touch_d.GetY()};
  const Point2d top_right_touch_g  = {top_right_touch_h.GetX() + 1.0, top_right_touch_h.GetY()};

  // Construct surface lines (weld object surface in torch plane) from surface points
  Line2d top_left_surface_line  = Line2d::FromPoints(top_left_touch_k, top_left_touch_l);
  Line2d top_right_surface_line = Line2d::FromPoints(top_right_touch_h, top_right_touch_g);
  Line2d left_groove_wall_line  = Line2d::FromPoints(left_wall_touch_b, left_wall_touch_j);
  Line2d right_groove_wall_line = Line2d::FromPoints(right_wall_touch_d, right_wall_touch_i);

  // Intersect surface vectors to get joint top corners
  auto top_left_corner_a  = top_left_surface_line.Intersect(left_groove_wall_line, false, false);
  auto top_right_corner_e = top_right_surface_line.Intersect(right_groove_wall_line, false, false);

  if (!top_left_corner_a.has_value()) {
    LOG_ERROR("Top left corner could not be determined");
    return {};
  }

  if (!top_right_corner_e.has_value()) {
    LOG_ERROR("Top right corner could not be determined");
    return {};
  }

  LOG_INFO("Computed torch plane points are:");
  LOG_INFO("A: ({},{})", top_left_corner_a->GetX(), top_left_corner_a->GetY());
  LOG_INFO("B: ({},{})", left_wall_touch_b.GetX(), left_wall_touch_b.GetY());
  LOG_INFO("D: ({},{})", right_wall_touch_d.GetX(), right_wall_touch_d.GetY());
  LOG_INFO("I: ({},{})", right_wall_touch_i.GetX(), right_wall_touch_i.GetY());
  LOG_INFO("J: ({},{})", left_wall_touch_j.GetX(), left_wall_touch_j.GetY());
  LOG_INFO("G: ({},{})", top_right_touch_g.GetX(), top_right_touch_g.GetY());
  LOG_INFO("H: ({},{})", top_right_touch_h.GetX(), top_right_touch_h.GetY());
  LOG_INFO("K: ({},{})", top_left_touch_k.GetX(), top_left_touch_k.GetY());
  LOG_INFO("L: ({},{})", top_left_touch_l.GetX(), top_left_touch_l.GetY());
  LOG_INFO("E: ({},{})", top_right_corner_e->GetX(), top_right_corner_e->GetY());

  // Compute the top center point
  const Eigen::Vector2d top_center_point = HALF * (top_left_corner_a->ToVector() + top_right_corner_e->ToVector());

  return common::Point{.horizontal = top_center_point(0), .vertical = top_center_point(1)};
}

// depth_c is the vertical distance from the touch points to the groove top
inline auto GenerateCalibrationDots(const GridConfiguration& grid_config, double c_left_x, double c_right_x, double c_z,
                                    double depth_c) -> std::vector<GridPoint> {
  const double c_width  = c_left_x - c_right_x;
  const double x_center = std::midpoint(c_left_x, c_right_x);

  // A is a rectangle with base corners at c_right and c_left
  // and top corners a distance margin_top above the top surface.
  const double a_xmin = c_right_x;
  const double a_xmax = c_left_x;
  const double a_zmin = c_z;
  const double a_zmax = c_z + depth_c + grid_config.margin_top;

  // B is a rectangle with base at the top of rectangle A
  // The height is margin_z, width extends margin_x outside
  // rectangle A on both sides.
  const double b_xmin = c_right_x - grid_config.margin_x;
  const double b_xmax = c_left_x + grid_config.margin_x;
  const double b_zmin = a_zmax;
  const double b_zmax = b_zmin + grid_config.margin_z;

  // This lambda determines if a point is inside the combined region of A & B
  auto inside_area = [&](double px, double pz) -> bool {
    const bool in_a = (px >= a_xmin) && (px <= a_xmax) && (pz >= a_zmin) && (pz <= a_zmax);
    const bool in_b = (px >= b_xmin) && (px <= b_xmax) && (pz >= b_zmin) && (pz <= b_zmax);
    return in_a || in_b;
  };

  // The initial value of the spacing is set very large
  // then the algorithm reduces the value with REDUCTION_FACTOR
  // until the number of dots exceed the target.
  double spacing = HALF * c_width;
  std::vector<GridPoint> ordered_points;

  while (true) {
    std::vector<GridPoint> raw_points;
    auto layout = std::make_shared<GridLayout>(GridLayout{.x_center = x_center, .z_start = c_z, .spacing = spacing});

    for (int index_z = 0;; ++index_z) {
      const double pz = layout->z_start + (index_z * layout->spacing);
      if (pz > (c_z + depth_c + grid_config.margin_top + grid_config.margin_z)) {
        break;
      }

      const int max_index_x = static_cast<int>((c_width / 2.0 + grid_config.margin_x) / spacing) + 1;
      for (int index_x = -max_index_x; index_x <= max_index_x; ++index_x) {
        const double px = layout->x_center + (index_x * layout->spacing);

        if (index_z == 0) {
          if (px > (c_left_x - grid_config.margin_c) || px < (c_right_x + grid_config.margin_c)) {
            // Avoid the region close to c_right and c_left which is near the wall
            continue;
          }
        }

        if (inside_area(px, pz)) {
          raw_points.emplace_back(GridPoint{.index_x = index_x, .index_z = index_z, .layout = layout});
        }
      }
    }

    if (raw_points.size() >= grid_config.target_nr_gridpoints) {
      // The target number of dots has been reached.
      // Sort the dots according to a snake pattern and break

      std::map<int, std::vector<GridPoint>> rows;
      for (const auto& point : raw_points) {
        rows[point.index_z].push_back(point);
      }

      ordered_points.clear();
      size_t row_index = 0;

      for (auto& [row_z, row_points] : rows) {
        if (row_index % 2 == 0) {
          // Sort in order right to left. Note that x increases from right to left(!)
          // That means low index_x values should be sorted before high index_x values
          std::ranges::sort(row_points,
                            [](const GridPoint& lhs, const GridPoint& rhs) { return lhs.index_x < rhs.index_x; });
        } else {
          // Sort in order left to right
          std::ranges::sort(row_points,
                            [](const GridPoint& lhs, const GridPoint& rhs) { return lhs.index_x > rhs.index_x; });
        }
        ordered_points.insert(ordered_points.end(), row_points.begin(), row_points.end());
        ++row_index;
      }

      break;
    }

    spacing *= REDUCTION_FACTOR;
  }

  return ordered_points;
}

}  // namespace calibration
