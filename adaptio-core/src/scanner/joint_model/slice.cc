#include "core/scanner/joint_model/slice.h"

#include <algorithm>
#include <boost/math/statistics/linear_regression.hpp>
#include <Eigen/Eigen>
#include <expected>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <optional>

#include "core/logging/application_log.h"
#include "core/math/value.h"
#include "core/scanner/joint_model.h"

using core::image::PlaneCoordinates;
using core::image::WorkspaceCoordinates;
using core::scanner::Point;
using core::scanner::joint_model::Slice;

// #define VISUAL_DEBUG_OUTPUT 1

const double MAX_HORIZONTAL_MOVEMENT    = 0.002;
const double MAX_WALL_HEIGHT_DIFFERENCE = 0.006;
const double DEEP_ENOUGH_FOR_SPEC_CHECK = 0.007;
/*
   _____ _ _
  / ____| (_)
 | (___ | |_  ___ ___
  \___ \| | |/ __/ _ \
  ____) | | | (_|  __/
 |_____/|_|_|\___\___|
*/

auto Slice::FromSnake(const core::image::WorkspaceCoordinates& snake, const core::scanner::JointProperties& properties,
                      const std::optional<JointProfile>& median_profile, bool history_joint_properties,
                      bool& found_out_of_spec_joint_width, std::optional<std::tuple<Point, Point>> history_abw0_abw6)
    -> std::expected<std::tuple<ABWPoints, uint64_t>, JointModelErrorCode> {
  auto left_angle                  = JointModel::LPCSFromWeldObjectAngle(properties.left_joint_angle) - M_PI_2;
  auto right_angle                 = M_PI_2 - JointModel::LPCSFromWeldObjectAngle(properties.right_joint_angle);
  auto joint_width                 = properties.upper_joint_width * 1.0e-3;
  auto upper_joint_width_tolerance = properties.upper_joint_width_tolerance * 1.0e-3;
  auto min_joint_width             = joint_width - (0.5 * upper_joint_width_tolerance);
  auto max_joint_width             = joint_width + (0.5 * upper_joint_width_tolerance);

  // Check that history ABW0 and ABW6 is within snake horizontally
  std::optional<Point> abw0_old;
  std::optional<Point> abw6_old;
  if (history_abw0_abw6) {
    auto [abw0, abw6] = history_abw0_abw6.value();
    abw0_old = FindIntersection(abw0.x, snake);
    abw6_old = FindIntersection(abw6.x, snake);

    if (!abw0_old || !abw6_old) {
      LOG_ERROR("Old abw0_x: {:.5f} old abw6_x {:.5f} snake {:.5f} {:.5f}", abw0.x, abw6.x, snake(0, 0),
                snake(0, snake.cols() - 1));
      return std::unexpected(JointModelErrorCode::FAULTY_HISTORY_DATA);
    }
  }
  auto angles = CalculateAngles(snake);
#if defined(VISUAL_DEBUG_OUTPUT)
  {
    std::ofstream fp("snake_test_angles.txt", std::ios::out | std::ios::trunc);
    for (auto& a : angles) {
      fp << a << " ";
    }
  }
#endif
  auto offset_distance = properties.offset_distance * 1e-3;
  auto num_walls_found = 0;
  auto maybe_abw0_and_abw1 =
      FindWall(angles, left_angle, properties)
          .and_then([snake, offset_distance,
                     median_profile](std::tuple<int, int> wall) -> std::optional<std::tuple<Point, Point>> {
            auto [start, stop] = wall;
            if (stop - start + 1 < 2 || start < 2) {
              return std::nullopt;
            }
            auto wall_line_segment    = JointModel::FitPoints(snake.block(0, start, 2, stop - start + 1), 0.0001);
            auto surface_line_segment = JointModel::FitPoints(snake.block(0, 0, 2, start), 0.0001);
            auto bottom               = snake.block(0, stop, 3, snake.cols() - stop);
            return wall_line_segment.intersection(surface_line_segment)
                .transform([](std::tuple<double, double> intersection) -> Point {
                  return Point{std::get<0>(intersection), std::get<1>(intersection)};
                })
                .and_then([bottom, wall_line_segment, offset_distance,
                           median_profile](Point abw0) -> std::optional<std::tuple<Point, Point>> {
                  const auto previous_abw0_x =
                      median_profile.transform([](JointProfile p) { return p.points[0].x; }).value_or(abw0.x);
                  if (fabs(abw0.x - previous_abw0_x) > MAX_HORIZONTAL_MOVEMENT) {
                    LOG_TRACE("Rejected left wall. {:.5f} vs {:.5f}. ", abw0.x, previous_abw0_x);
                    return std::nullopt;
                  } else {
                    return FindIntersection(wall_line_segment.TranslatedHorizontally(offset_distance), bottom)
                        .transform([abw0, offset_distance](Point intersection) -> std::tuple<Point, Point> {
                          return std::make_tuple(abw0, intersection.TranslatedHorizontally(-offset_distance));
                        });
                  }
                });
          });
  auto maybe_abw5_and_abw6 =
      FindWall(angles, right_angle, properties)
          .and_then([snake, offset_distance,
                     median_profile](std::tuple<int, int> wall) -> std::optional<std::tuple<Point, Point>> {
            auto [start, stop] = wall;
            if (stop - start + 1 < 2 || snake.cols() - stop < 2) {
              return std::nullopt;
            }
            auto wall_line_segment    = JointModel::FitPoints(snake.block(0, start, 2, stop - start + 1), 0.0001);
            auto surface_line_segment = JointModel::FitPoints(snake.block(0, stop, 2, snake.cols() - stop), 0.0001);
            auto bottom               = snake.block(0, 0, 3, start);
            return wall_line_segment.intersection(surface_line_segment)
                .transform([](std::tuple<double, double> intersection) -> Point {
                  return Point{std::get<0>(intersection), std::get<1>(intersection)};
                })
                .and_then([bottom, wall_line_segment, offset_distance,
                           median_profile](Point abw6) -> std::optional<std::tuple<Point, Point>> {
                  const auto previous_abw6_x =
                      median_profile.transform([](JointProfile p) { return p.points[6].x; }).value_or(abw6.x);
                  if (fabs(abw6.x - previous_abw6_x) > MAX_HORIZONTAL_MOVEMENT) {
                    LOG_TRACE("Rejected right wall. {:.5f} vs {:.5f}. ", abw6.x, previous_abw6_x);
                    return std::nullopt;
                  } else {
                    return FindIntersection(wall_line_segment.TranslatedHorizontally(-offset_distance), bottom)
                        .transform([abw6, offset_distance](Point intersection) -> std::tuple<Point, Point> {
                          return std::make_tuple(intersection.TranslatedHorizontally(offset_distance), abw6);
                        });
                  }
                });
          });
  if (maybe_abw0_and_abw1.has_value()) {
    num_walls_found++;
  }
  if (maybe_abw5_and_abw6.has_value()) {
    num_walls_found++;
  }

  if (num_walls_found == 2) {
    const auto [abw0, abw1] = maybe_abw0_and_abw1.value();
    const auto [abw5, abw6] = maybe_abw5_and_abw6.value();
    const auto width        = abw6.x - abw0.x;
    // Check that the distance between the walls is within tolerance
    if (width < min_joint_width || width > max_joint_width) {
      const bool high_confidence_walls =
          median_profile
              .transform([](JointProfile p) -> std::tuple<double, double> {
                const double left_height  = p.points[0].y - p.points[1].y;
                const double right_height = p.points[6].y - p.points[5].y;
                return {left_height, right_height};
              })
              .transform([](std::tuple<double, double> depths) -> bool {
                auto [left, right] = depths;
                return (left >= DEEP_ENOUGH_FOR_SPEC_CHECK && right >= DEEP_ENOUGH_FOR_SPEC_CHECK);
              })
              .value_or(false);
      if (found_out_of_spec_joint_width || high_confidence_walls) {
        // Assume that the joint walls were probably detected ok, and it is the joint width that is wrong
        // Also if we previously detected an out of spec joint we assume this is still the case (since
        // we will lose the median profile after a short while, before triggering the scanner timeout)
        found_out_of_spec_joint_width = true;
        return std::unexpected(JointModelErrorCode::JOINT_WIDTH_OUT_OF_TOLERANCE);
      }

      LOG_TRACE("Disregarding walls due to width out of tolerance: {:.5f} to {:.5f} = {:5.f}. Limits {:.5f}/{:.5f}",
                abw0.x, abw6.x, width, min_joint_width, max_joint_width);
      if (history_joint_properties) {
        num_walls_found = 0;
      }
    } else {
      found_out_of_spec_joint_width = false;
    }
  } else if (!abw0_old || !abw6_old) {
    // If we do not have joint properties for earlier turns we do not try to find ABWs when zero or one wall
    // is found
    return std::unexpected(JointModelErrorCode::TWO_WALLS_NOT_FOUND);
  }

  // If we have one corner and the previous profile, use that.
  Point abw0;
  Point abw1;
  Point abw5;
  Point abw6;
  if (num_walls_found == 1) {
    if (maybe_abw0_and_abw1) {
      std::tie(abw0, abw1) = maybe_abw0_and_abw1.value();
      abw6                 = abw6_old.value();
      auto maybe_abw5 = FindIntersection(LineSegment(abw6, right_angle).TranslatedHorizontally(-offset_distance), snake)
                            .transform([offset_distance](Point intersection) {
                              return intersection.TranslatedHorizontally(offset_distance);
                            });
      if (!maybe_abw5) {
        // Should not happen
        return std::unexpected(JointModelErrorCode::INVALID_SNAKE);
      }
      abw5 = maybe_abw5.value();
    } else if (maybe_abw5_and_abw6) {
      std::tie(abw5, abw6) = maybe_abw5_and_abw6.value();
      abw6                 = abw0_old.value();
      auto maybe_abw1 = FindIntersection(LineSegment(abw0, left_angle).TranslatedHorizontally(offset_distance), snake)
                            .transform([offset_distance](Point intersection) {
                              return intersection.TranslatedHorizontally(-offset_distance);
                            });
      if (!maybe_abw1) {
        // Should not happen
        return std::unexpected(JointModelErrorCode::INVALID_SNAKE);
      }
      abw1 = maybe_abw1.value();
    }
    if (abw0.x > abw1.x || abw6.x < abw5.x) {
      return std::unexpected(JointModelErrorCode::INVALID_SNAKE);
    }
    if (median_profile.has_value()) {
      auto p = median_profile.value();
      if (fabs((abw0.y - abw1.y) - (p.points[0].y - p.points[1].y)) > MAX_WALL_HEIGHT_DIFFERENCE ||
          fabs((abw6.y - abw5.y) - (p.points[6].y - p.points[5].y)) > MAX_WALL_HEIGHT_DIFFERENCE) {
        return std::unexpected(JointModelErrorCode::INVALID_WALL_HEIGHT_DIFFERENCE);
      }
    }
  } else {
    // No walls found
    abw0 = abw0_old.value();
    abw1 = FindIntersection(LineSegment(abw0, left_angle).TranslatedHorizontally(offset_distance), snake)
               .transform([offset_distance](Point p) { return p.TranslatedHorizontally(-offset_distance); })
               .value_or(abw0);
    abw6 = abw6_old.value();
    abw5 = FindIntersection(LineSegment(abw6, right_angle).TranslatedHorizontally(-offset_distance), snake)
               .transform([offset_distance](Point p) { return p.TranslatedHorizontally(offset_distance); })
               .value_or(abw6);
  }

  if (snake.cols() > 5) {
    const auto left_surface_angle = atan2(0.2 * snake.block(1, 0, 1, 5).sum() - abw0.y, abw0.x - snake(0, 2));
    const auto right_surface_angle =
        atan2(0.2 * snake.block(1, snake.cols() - 5, 1, 5).sum() - abw6.y, snake(0, snake.cols() - 1) - abw6.x);
    if (fabs(left_surface_angle) > 0.5 * properties.surface_angle_tolerance ||
        fabs(right_surface_angle) > 0.5 * properties.surface_angle_tolerance) {
      return std::unexpected(JointModelErrorCode::SURFACE_ANGLE_TOLERANCE_EXCEEDED);
    }
  }

  LOG_TRACE(
      "Num walls ({}) History joint ({}) ABW0 ({:.5f}, {:.5f}) ABW1 ({:.5f}, {:.5f}) ABW5 ({:.5f}, {:.5f}) "
      "ABW6 ({:.5f}, {:.5f})",
      num_walls_found, history_joint_properties ? "yes" : "no", abw0.x, abw0.y, abw1.x, abw1.y, abw5.x, abw5.y, abw6.x,
      abw6.y);

  ABWPoints points = {abw0, abw1, abw0, abw0, abw0, abw5, abw6};
  // Find bottom intersect
  const auto quarter_width = 0.25 * (abw5.x - abw1.x);
  for (int pt = 1; pt < 4; pt++) {
    auto x         = quarter_width * static_cast<double>(pt) + abw1.x;
    points[pt + 1] = FindIntersection(x, snake).value_or(Point{x, (abw1.y * pt + abw5.y * (4 - pt)) * 0.25});
  }

  return std::make_tuple(points, num_walls_found);
}

auto Slice::FindInCoordinates(const core::image::WorkspaceCoordinates& snake,
                              const std::function<bool(const Eigen::Vector3d&)>& lambda) -> std::optional<int> {
  for (int i = 0; i < snake.cols(); i++) {
    if (lambda(snake.col(i))) {
      return i;
    }
  }
  return std::nullopt;
}

auto Slice::FindWall(const std::vector<double>& angles, double angle, const JointProperties& properties)
    -> std::optional<std::tuple<int, int>> {
  std::vector<int> indices;
  const double low  = angle - properties.groove_angle_tolerance;
  const double high = angle + properties.groove_angle_tolerance;

  for (int i = 0; i < angles.size(); i++) {
    if (angles[i] >= low && angles[i] <= high) {
      indices.push_back(i);
    }
  }
  if (indices.size() == 0) {
    return std::nullopt;
  }

  // Get all indices close to the median.
  // Since the list is already sorted in x, the median index is just the middle element.
  // Go to earlier/later indices if they are less than 5 away
  int start = indices.size() / 2 - 1;
  while (start >= 0 && indices[start] + 5 > indices[start + 1]) {
    start--;
  }
  int stop = indices.size() / 2 + 1;
  while (stop < indices.size() && indices[stop - 1] + 5 > indices[stop]) {
    stop++;
  }

  if (indices[stop - 1] - indices[start + 1] > 2) {
    return {
        {indices[start + 1], indices[stop - 1]}
    };
  } else {
    return std::nullopt;
  }
}

auto Slice::FindIntersection(const LineSegment& line, const WorkspaceCoordinates& snake) -> std::optional<Point> {
  if (snake.cols() == 0) {
    return std::nullopt;
  }
  const auto y3 = snake.row(1).minCoeff();
  const auto y4 = snake.row(1).maxCoeff();
  const auto x3 = (y3 - line.m) / line.k;
  const auto x4 = (y4 - line.m) / line.k;
  for (int segment = 0; segment < snake.cols() - 1; segment++) {
    const auto x1 = snake(0, segment);
    const auto y1 = snake(1, segment);
    const auto x2 = snake(0, segment + 1);
    const auto y2 = snake(1, segment + 1);
    const auto mt = (x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4);
    const auto n  = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    const auto mu = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3));
    if (((n > 0) && (mt >= 0.0 && mt <= n && mu >= 0.0 && mu <= n)) ||
        ((n < 0) && (mt <= 0.0 && mt > n && mu <= 0.0 && mu > n))) {
      const auto t = mt / n;
      return Point{x1 + t * (x2 - x1), y1 + t * (y2 - y1)};
    }
  }
  return std::nullopt;
}

auto Slice::FindIntersection(const double x, const WorkspaceCoordinates& snake) -> std::optional<Point> {
  for (int segment = 0; segment < snake.cols() - 1; segment++) {
    auto x0 = snake(0, segment);
    auto x1 = snake(0, segment + 1);
    if (x >= x0 && x <= x1 && x0 != x1) {
      auto y0        = snake(1, segment);
      auto y1        = snake(1, segment + 1);
      auto k_segment = (y1 - y0) / (x1 - x0);
      auto m_segment = y0 - k_segment * x0;
      return Point{x, k_segment * x + m_segment};
    }
  }
  return std::nullopt;
}

auto Slice::FindJoint(const core::image::WorkspaceCoordinates& snake, const std::vector<double>& angles,
                      const double joint_width, const double left_angle, const double right_angle,
                      const std::optional<Point> previous_abw0, const JointProperties& current_joint_properties)
    -> std::tuple<Point, Point, Point, Point> {
  auto get_standard_deviation = [snake, angles, joint_width](const double start_x) -> std::optional<double> {
    long start_index, stop_index;
    for (start_index = 0; start_index < snake.cols() && snake(0, start_index) < start_x; start_index++);
    for (stop_index = start_index; stop_index < snake.cols() && snake(0, stop_index) < start_x + joint_width;
         stop_index++);
    if (stop_index - start_index < 2) {
      return std::nullopt;
    }
    return core::math::value::StandardDeviation(angles, start_index, stop_index);
  };

  const long NUM_SAMPLES = 20;
  auto x_min =
      previous_abw0.transform([](Point p) { return p.x - 6.0 * MAX_HORIZONTAL_MOVEMENT; }).value_or(snake(0, 0));
  auto x_max = previous_abw0.transform([](Point p) { return p.x + 6.0 * MAX_HORIZONTAL_MOVEMENT; })
                   .value_or(snake(0, snake.cols() - 1) - joint_width);
  double step = (x_max - x_min) / NUM_SAMPLES;

  do {
    auto best       = 0.0;
    auto best_index = -1;
    for (long i = 0; i < NUM_SAMPLES; i++) {
      auto stdev_here = get_standard_deviation(x_min + step * i).value_or(0.0);
      if (stdev_here >= best) {
        best       = stdev_here;
        best_index = i;
      }
    }

    // Set a new x_min and x_max surrounding the best index.
    const auto range = 0.1 * (x_max - x_min);
    x_min            = x_min + step * best_index - 0.5 * range;
    x_max            = x_min + range;
    step             = (x_max - x_min) / NUM_SAMPLES;
  } while (step > 0.0001);

  LOG_TRACE("No walls found. Located maximum angle stdev at x = [{}, {}]", x_min, x_min + joint_width);

  const auto offset_distance = current_joint_properties.offset_distance * 1.0e-3;

  const auto min_joint_width =
      (current_joint_properties.upper_joint_width - 0.5 * current_joint_properties.upper_joint_width_tolerance) *
      1.0e-3;
  const auto max_joint_width =
      (current_joint_properties.upper_joint_width + 0.5 * current_joint_properties.upper_joint_width_tolerance) *
      1.0e-3;

  const auto abw0 =
      FindInCoordinates(snake, [x_min](const Eigen::Vector3d& col) { return col(0, 0) > x_min; })
          .and_then([x_min, snake, joint_width, min_joint_width,
                     max_joint_width](int start_index) -> std::optional<Point> {
            if (start_index < 2) {
              return std::nullopt;
            }
            auto surface = JointModel::FitPoints(snake.block(0, 0, 2, start_index), 0.0001);
            const auto delta =
                std::clamp(x_min - surface.x_limits.max, -0.5 * MAX_HORIZONTAL_MOVEMENT, 0.5 * MAX_HORIZONTAL_MOVEMENT);
            const auto x =
                std::clamp(x_min - delta, x_min + joint_width - max_joint_width, x_min + joint_width - min_joint_width);
            return {
                Point{x, surface.k * x + surface.m}
            };
          })
          .value_or(FindIntersection(x_min, snake).value_or(Point{x_min, snake(1, 0)}));

  const auto abw6 =
      FindInCoordinates(snake,
                        [abw0, joint_width](const Eigen::Vector3d& col) { return col(0, 0) > abw0.x + joint_width; })
          .and_then([abw0, snake, joint_width, min_joint_width,
                     max_joint_width](int start_index) -> std::optional<Point> {
            if (snake.cols() - start_index < 2) {
              return std::nullopt;
            }
            auto surface = JointModel::FitPoints(snake.block(0, start_index, 2, snake.cols() - start_index), 0.0001);
            const auto delta = std::clamp(abw0.x + joint_width - surface.x_limits.min, -0.5 * MAX_HORIZONTAL_MOVEMENT,
                                          0.5 * MAX_HORIZONTAL_MOVEMENT);
            const auto x = std::clamp(abw0.x + joint_width - delta, abw0.x + min_joint_width, abw0.x + max_joint_width);
            return {
                Point{x, surface.k * x + surface.m}
            };
          })
          .value_or(FindIntersection(x_min + joint_width, snake)
                        .value_or(Point{x_min + joint_width, snake(1, snake.cols() - 1)}));

  const auto abw1 = FindIntersection(LineSegment(abw0, left_angle).TranslatedHorizontally(offset_distance), snake)
                        .transform([offset_distance](Point p) { return p.TranslatedHorizontally(-offset_distance); })
                        .value_or(abw0);
  const auto abw5 = FindIntersection(LineSegment(abw6, right_angle).TranslatedHorizontally(-offset_distance), snake)
                        .transform([offset_distance](Point p) { return p.TranslatedHorizontally(offset_distance); })
                        .value_or(abw6);

  return {abw0, abw1, abw5, abw6};
}

auto Slice::CalculateAngles(const WorkspaceCoordinates& snake) -> std::vector<double> {
  auto angles   = std::vector<double>(snake.cols() - 1);
  const auto dx = snake.row(0).segment(1, snake.cols() - 1) - snake.row(0).segment(0, snake.cols() - 1);
  const auto dy = snake.row(1).segment(1, snake.cols() - 1) - snake.row(1).segment(0, snake.cols() - 1);
  for (int i = 0; i < snake.cols() - 1; i++) {
    angles[i] = atan2(dy(i), dx(i));
  }

  return angles;
}
