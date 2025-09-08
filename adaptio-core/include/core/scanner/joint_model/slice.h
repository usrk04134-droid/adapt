#pragma once

#include <boost/circular_buffer.hpp>
#include <Eigen/Eigen>
#include <expected>
#include <fstream>
#include <optional>
#include <tuple>
#include <vector>

#include "core/image/camera_model.h"
#include "core/image/image_types.h"
#include "core/scanner/joint_model.h"
#include "core/scanner/joint_model/snake.h"
#include "core/scanner/scanner_configuration.h"

namespace core::scanner::joint_model {

struct Slice {
  std::vector<double> x;
  std::vector<double> y;

  uint8_t min_pixel_value;
  uint8_t max_pixel_value;

  /**
   * Finds the "snake", a contiguous list of coordinates starting from either
   * the left or the right side.
   *
   * @param snake                         The snake i.e. the line
   * @param properties                    The joint properties. Can be joint properties in web hmi or
   *                                      joint properties from earlier turns
   * @param median_profile                The median slice
   * @param history_joint_properties      Indicates if history joint properties is used
   * @param found_out_of_spec_joint_width Indicates i joint width is out of spec
   * @return The snake (a Snake instance), or nullopt on failure.
   */
  static auto FromSnake(const core::image::WorkspaceCoordinates& snake,
                        const core::scanner::JointProperties& properties,
                        const std::optional<JointProfile>& median_profile, bool history_joint_properties,
                        bool& found_out_of_spec_joint_width, std::optional<std::tuple<Point, Point>> history_abw0_abw6)
      -> std::expected<std::tuple<ABWPoints, uint64_t>, JointModelErrorCode>;

 protected:
  static auto CalculateAngles(const core::image::WorkspaceCoordinates& snake) -> std::vector<double>;

  /**
   * Helper function to find a coordinate matching a condition.
   *
   * @param snake  Centroids in LPCS coordinates
   * @param lambda Function for determining the match condition
   * @return The first index for which the condition returns true.
   */
  static auto FindInCoordinates(const core::image::WorkspaceCoordinates& snake,
                                const std::function<bool(const Eigen::Vector3d&)>& lambda) -> std::optional<int>;

  static auto FindWall(const std::vector<double>& angles, double angle, const JointProperties& properties)
      -> std::optional<std::tuple<int, int>>;

  /**
   * Locate an intersection between the snake in LPCS coordinates and a LineSegment.
   *
   * @param line  The line.
   * @param snake The (partial) snake to look through.
   * @return The first intersection point, or nullopt if no intersection was found.
   */
  static auto FindIntersection(const LineSegment& line, const core::image::WorkspaceCoordinates& snake)
      -> std::optional<Point>;

  /**
   * Locate an intersection between the snake in LPCS coordinates and a vertical line.
   *
   * @param x     The x coordinate of the vertical line.
   * @param snake The (partial) snake to look through.
   * @return The first intersection point, or nullopt if no intersection was found.
   */
  static auto FindIntersection(const double x, const core::image::WorkspaceCoordinates& snake) -> std::optional<Point>;

  /**
   * Find the joint and extract abw0,1,5,6. This is the fall-back method
   * used if the more structured approach of finding the walls failed.
   * Based on finding the range of angles with the highest standard deviation.
   *
   * @param snake                    The snake to look through.
   * @param angles                   The list of angles.
   * @param joint_width              The best guess joint width
   * @param left_angle               The best guess left angle.
   * @param right_angle              The best guess right angle.
   * @param previous_abw0            Previous abw0. Used to check that we don't move too far each frame.
   * @param current_joint_properties The actual joint properties
   * @return A tuple of 4 abw points.
   */
  static auto FindJoint(const core::image::WorkspaceCoordinates& snake, const std::vector<double>& angles,
                        const double joint_width, const double left_angle, const double right_angle,
                        const std::optional<Point> previous_abw0, const JointProperties& current_joint_properties)
      -> std::tuple<Point, Point, Point, Point>;
};
}  // namespace core::scanner::joint_model
