#pragma once

#include <boost/circular_buffer.hpp>
#include <Eigen/Eigen>
#include <expected>
#include <fstream>
#include <optional>
#include <tuple>
#include <vector>

#include "scanner/image/camera_model.h"
#include "scanner/image/image_types.h"
#include "scanner/joint_model/joint_model.h"
#include "scanner/joint_model/snake.h"
#include "scanner/scanner_configuration.h"

namespace scanner::joint_model {

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
   * @param found_out_of_spec_joint_width Indicates i joint width is out of spec
   * @param joint_properties_updated      Indicates if joint properties is updated i.e. a correct width
   *                                      and a smaller tolerance
   * @param use_approximation             Idicates if only use approximation data
   * @param approximated_abw0_abw6        Approximation of abw0/abw6 if available
   * @return The snake (a Snake instance), or nullopt on failure.
   */
  static auto FromSnake(const image::WorkspaceCoordinates& snake, const JointProperties& properties,
                        const std::optional<JointProfile>& median_profile, bool& found_out_of_spec_joint_width,
                        bool joint_properties_updated, bool use_approximation,
                        std::optional<std::tuple<double, double>> approx_abw0_abw6)
      -> std::expected<std::tuple<ABWPoints, uint64_t, bool>, JointModelErrorCode>;

 protected:
  static auto CalculateAngles(const image::WorkspaceCoordinates& snake) -> std::vector<double>;

  /**
   * Helper function to find a coordinate matching a condition.
   *
   * @param snake  Centroids in LPCS coordinates
   * @param lambda Function for determining the match condition
   * @return The first index for which the condition returns true.
   */
  static auto FindInCoordinates(const image::WorkspaceCoordinates& snake,
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
  static auto FindIntersection(const LineSegment& line, const image::WorkspaceCoordinates& snake)
      -> std::optional<Point>;

  /**
   * Locate an intersection between the snake in LPCS coordinates and a vertical line.
   *
   * @param x     The x coordinate of the vertical line.
   * @param snake The (partial) snake to look through.
   * @return The first intersection point, or nullopt if no intersection was found.
   */
  static auto FindIntersection(const double x, const image::WorkspaceCoordinates& snake) -> std::optional<Point>;

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
  static auto FindJoint(const image::WorkspaceCoordinates& snake, const std::vector<double>& angles,
                        const double joint_width, const double left_angle, const double right_angle,
                        const std::optional<Point> previous_abw0, const JointProperties& current_joint_properties)
      -> std::tuple<Point, Point, Point, Point>;

  /**
   * Position approximated abw0/abw6 on snake and from those calculate abw1/abw5
   *
   * @param snake                    The snake to look through.
   * @param abw0                     Approximated abw0.
   * @param abw6                     Approximated abw6.
   * @param left_angle               The best guess left angle.
   * @param right_angle              The best guess right angle.
   * @param offset_distance          Offset distance used to position abw1/abw5.
   * @return A tuple of 4 abw points.
   */
  static auto PositionPointsOnSnake(const image::WorkspaceCoordinates& snake,
                                    std::optional<std::tuple<double, double>>& approx_abw0_abw6, double left_angle,
                                    double right_angle, double offset_distance)
      -> std::optional<std::tuple<Point, Point, Point, Point>>;
};
}  // namespace scanner::joint_model
