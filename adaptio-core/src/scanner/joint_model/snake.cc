#include "core/scanner/joint_model/snake.h"

#include <algorithm>
#include <boost/math/statistics/linear_regression.hpp>
#include <Eigen/Eigen>
#include <expected>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <optional>

#include "core/image/image_types.h"
#include "core/logging/application_log.h"
#include "core/math/centroids.h"
#include "core/math/value.h"
#include "core/scanner/joint_model.h"

using core::image::Image;
using core::image::PlaneCoordinates;
using core::image::WorkspaceCoordinates;
using core::scanner::Point;
using core::scanner::joint_model::Snake;

// #define VISUAL_DEBUG_OUTPUT 1

const int MAX_ALLOWED_SIDE_GAP           = 600;
const double ANGLE_INSENSITIVITY         = 0.5;
const double SURFACE_ANGLE_INSENSITIVITY = 0.9;
const double MOVE_DISTANCE               = 3.0;
const int NUM_COLUMNS_TO_LOOK_FOR_START  = 9;
const int PIXEL_RANGE_INSET              = 10;

/*
 ____              _
/ ___| _ __   __ _| | _____
\___ \| '_ \ / _` | |/ / _ \
 ___) | | | | (_| |   <  __/
|____/|_| |_|\__,_|_|\_\___|
*/

auto Snake::FromImage(const image::Image& image, const std::optional<image::RawImageData>& mask, uint8_t threshold)
    -> std::optional<Snake> {
  auto left_snake = Snake::FromImage(image, mask, threshold, StartingPoint::Left, static_cast<double>(image.StopCol()));
  auto right_snake =
      Snake::FromImage(image, mask, threshold, StartingPoint::Right, static_cast<double>(image.StartCol()));

  auto snake = Snake::Merge(left_snake, right_snake).and_then([image](Snake s) -> std::optional<Snake> {
    const auto width = static_cast<double>(image.Cols());
    if (s.x.empty() || (s.x.front() - image.StartCol()) > 0.33 * width ||
        (s.x.back() - image.StartCol()) < 0.66 * width) {
      return std::nullopt;
    } else {
      auto values = std::vector<uint8_t>();
      for (size_t i = 0; i < s.x.size(); i++) {
        const long y = static_cast<long>(s.y[i]);
        const long x = static_cast<long>(s.x[i]);

        if (x < 1 || x >= image.Data().cols() - 1 || y < 1 || y >= image.Data().rows() - 1) {
          continue;
        }
        uint64_t total     = 0;
        uint64_t num_total = 0;
        for (long dy = -1; dy <= 1; dy++) {
          for (long dx = -1; dx <= 1; dx++) {
            total += image.Data()(y + dy, x + dx);
            num_total++;
          }
        }
        const auto value = static_cast<uint8_t>(total / num_total);
        values.push_back(value);
      }

      std::nth_element(values.begin(), values.begin() + PIXEL_RANGE_INSET, values.end());
      s.min_pixel_value = values[PIXEL_RANGE_INSET];
      std::nth_element(values.begin(), values.end() - PIXEL_RANGE_INSET, values.end());
      s.max_pixel_value = values[values.size() - PIXEL_RANGE_INSET];
      return s;
    }
  });

  return snake;
}

auto Snake::FromImage(const Image& image, const std::optional<image::RawImageData>& mask, uint8_t threshold,
                      enum StartingPoint side, double horizontal_limit) -> std::optional<Snake> {
  const int SNAKE_LOOK_WIDTH                              = 25;
  const int FAST_SNAKE_LOOK_HEIGHT                        = 17;
  const int NUM_POINTS_TO_ERASE_WHEN_LEAVING_SURFACE_MODE = 6;

  return FindStartColumn(image, threshold, side)
      .and_then([image, threshold, side](int column) -> std::optional<Point> {
        // 1. Find centroids for this column and the next 10 columns
        // 2. Get median y
        auto rows = std::vector<double>();
        for (int dx = 0; dx < NUM_COLUMNS_TO_LOOK_FOR_START; dx++) {
          int c;
          switch (side) {
            case StartingPoint::Left:
              c = column + dx;
              break;
            case StartingPoint::Right:
              c = column - dx;
              break;
          }
          if (c >= 0 && c < image.Data().cols()) {
            auto centroid =
                GetCentroidForSingleColumn(image.Data().col(c), math::CentroidSearchDirection::Reversed, threshold);
            if (centroid.has_value()) {
              rows.push_back(centroid.value());
            }
          }
        }
        if (rows.size() > 0) {
          const auto row = core::math::value::FindMedian(rows.begin(), rows.end(), rows.size());
          return {
              Point{static_cast<double>(column), row}
          };
        } else {
          return std::nullopt;
        }
      })
      .and_then([image, mask, side, threshold, horizontal_limit](struct Point starting_point) {
        struct Snake snake;
        const double h    = static_cast<double>(SNAKE_LOOK_WIDTH / 2);
        double angle      = (side == StartingPoint::Left) ? 0.0 : M_PI;
        int lost_track    = 0;
        bool surface_mode = true;

        struct Point current_point = starting_point;
        auto continue_ahead_x      = cos(angle);
        auto continue_ahead_y      = sin(angle);
        auto flip_180              = [](double a) { return a + ((a < 0) ? M_PI : (-M_PI)); };
        do {
          double sum_angle = 0.0;
          int64_t mass     = 0;

          const long current_px_x = static_cast<long>(round(current_point.x));
          const long current_px_y = static_cast<long>(round(current_point.y));
          const long y_limit      = surface_mode ? (FAST_SNAKE_LOOK_HEIGHT / 2) : (SNAKE_LOOK_WIDTH / 2);
          for (long dy = -y_limit; dy <= y_limit; dy++) {
            const auto y = current_px_y + dy;
            if (y < 0 || y >= image.Data().rows()) {
              continue;
            }
            for (long dx = -SNAKE_LOOK_WIDTH / 2; dx <= SNAKE_LOOK_WIDTH / 2; dx++) {
              const auto x  = current_px_x + dx;
              const auto r2 = dx * dx + dy * dy;
              if (x < image.StartCol() || x >= image.StopCol() || (mask.has_value() && mask.value()(y, x) == 0) ||
                  (dx == 0 && dy == 0) || r2 > (h * h)) {
                continue;
              }

              const auto pixel_value = static_cast<int64_t>(image.Data()(y, x)) - threshold;
              if (pixel_value > 0) {
                const auto raw_angle = atan2(dy, dx);
                const auto delta =
                    (fabs(raw_angle - angle) > M_PI) ? (flip_180(raw_angle) - flip_180(angle)) : (raw_angle - angle);
                if (fabs(delta) < (surface_mode ? 0.3 : 0.8) * M_PI_2) {
                  sum_angle += static_cast<double>(pixel_value) * delta;
                  mass      += pixel_value;
                }
              }
            }
          }

          // Surface mode does three things
          // 1. Decrease how much to the left and right we look
          // 2. Decrease how high an area around current_point we should look in
          // 3. React more slowly to changes in angle
          // We leave surface mode if either:
          // 1. No pixels in the search area are above the threshold (even once)
          // 2. The absolute value of the angle is more than 0.3 * M_PI_2
          bool leave_surface_mode = false;
          if (mass == 0) {
            if (surface_mode) {
              leave_surface_mode = true;
            } else {
              current_point = Point{current_point.x + MOVE_DISTANCE * continue_ahead_x,
                                    current_point.y + MOVE_DISTANCE * continue_ahead_y};
              lost_track++;
            }
          } else {
            const auto angle_compared_to_straight_ahead = sum_angle / static_cast<double>(mass);
            auto raw_angle                              = angle + angle_compared_to_straight_ahead;
            const auto sensitivity =
                (surface_mode && ((side == StartingPoint::Left && angle_compared_to_straight_ahead > 0) ||
                                  (side == StartingPoint::Right && angle_compared_to_straight_ahead < 0)))
                    ? SURFACE_ANGLE_INSENSITIVITY
                    : ANGLE_INSENSITIVITY;
            auto result      = atan2(sin(raw_angle) * (1.0 - sensitivity) + sin(angle) * sensitivity,
                                     cos(raw_angle) * (1.0 - sensitivity) + cos(angle) * sensitivity);
            auto going_right = cos(result) > 0.0;
            if ((side == StartingPoint::Left && going_right) || (side == StartingPoint::Right && !going_right)) {
              angle              = result;
              current_point      = current_point + Point{cos(raw_angle), sin(raw_angle)} * MOVE_DISTANCE;
              leave_surface_mode = surface_mode && (fabs(angle) > 0.3 * M_PI_2);
              snake.x.push_back(current_point.x);
              snake.y.push_back(current_point.y);
              lost_track = 0;
            } else {
              lost_track++;
            }
          }
          if (leave_surface_mode) {
            surface_mode = false;
            if (snake.x.size() > NUM_POINTS_TO_ERASE_WHEN_LEAVING_SURFACE_MODE) {
              snake.x.erase(snake.x.end() - NUM_POINTS_TO_ERASE_WHEN_LEAVING_SURFACE_MODE, snake.x.end());
              snake.y.erase(snake.y.end() - NUM_POINTS_TO_ERASE_WHEN_LEAVING_SURFACE_MODE, snake.y.end());
              current_point = Point{snake.x.back(), snake.y.back()};
            }
          }
        } while (lost_track < 5 && ((side == StartingPoint::Left && (current_point.x < horizontal_limit)) ||
                                    (side == StartingPoint::Right && (current_point.x > horizontal_limit))));

        return (snake.x.size() > 0) ? std::make_optional(snake) : std::nullopt;
      });
}

auto Snake::FindStartColumn(const Image& image, uint8_t threshold, enum StartingPoint side) -> std::optional<int> {
  const int first = (side == StartingPoint::Left) ? image.StartCol() : (image.StopCol() - 1);
  const int delta = (side == StartingPoint::Left) ? 1 : -1;
  const int last  = (side == StartingPoint::Left) ? image.StartCol() + MAX_ALLOWED_SIDE_GAP
                                                  : (image.StopCol() - MAX_ALLOWED_SIDE_GAP);

  for (int column = first; column != last; column += delta) {
    if (image.Data().col(column).maxCoeff() > threshold) {
      return std::make_optional(column);
    }
  }

  return std::nullopt;
}

auto Snake::Merge(const std::optional<Snake>& left_snake, const std::optional<Snake>& right_snake)
    -> std::optional<Snake> {
  if (left_snake.has_value() && right_snake.has_value()) {
    auto left             = left_snake.value();
    auto right            = right_snake.value();
    const auto left_ends  = left.x.back();
    const auto right_ends = right.x.back();
    // Check if there is more than 20 px gap in the middle.
    if (left_ends + 20 < right_ends) {
      return std::nullopt;
    }

    // Check if left terminates far from the starting point of the right
    if (left_ends + 20 < right.x.front()) {
      // If so, clip left to where right and left meet first (from the perspective of the right)
      for (int i = 0; i < right.x.size(); i++) {
        if (right.x[i] > left_ends) {
          continue;
        }
        int j;
        for (j = left.x.size() - 1; j > 0 && left.x[j] > right.x[i]; j--);
        if (fabs(left.y[j] - right.y[i]) < MOVE_DISTANCE) {
          left.x.erase(left.x.begin() + j, left.x.end());
          left.y.erase(left.y.begin() + j, left.y.end());
          break;
        }
      }
    }
    // Same, but for other side.
    if (left.x.front() + 20 < right_ends) {
      for (int i = 0; i < left.x.size(); i++) {
        if (left.x[i] < right_ends) {
          continue;
        }
        int j;
        for (j = right.x.size() - 1; j > 0 && right.x[j] < left.x[i]; j--);
        if (fabs(left.y[i] - right.y[j]) < MOVE_DISTANCE) {
          right.x.erase(right.x.begin() + j, right.x.end());
          right.y.erase(right.y.begin() + j, right.y.end());
          break;
        }
      }
    }
    if (left.x.size() == 0) {
      return right.Reversed();
    } else if (right.x.size() == 0) {
      return left;
    } else {
      return left.ConcatenateInReverse(right);
    }
  } else {
    return left_snake.or_else([right_snake]() { return right_snake.transform([](Snake r) { return r.Reversed(); }); });
  }
}

auto Snake::Reversed() const -> Snake {
  Snake s;

  s.x.insert(s.x.end(), std::reverse_iterator(x.end()), std::reverse_iterator(x.begin()));
  s.y.insert(s.y.end(), std::reverse_iterator(y.end()), std::reverse_iterator(y.begin()));
  return s;
}

auto Snake::ConcatenateInReverse(Snake other) const -> Snake {
  Snake s{x, y};
  const auto left_snake_end  = s.x.back();
  const auto right_snake_end = other.x.back();

  if (left_snake_end >= right_snake_end) {
    // Get overlapping x parts and average them smoothly
    const auto left_snake_overlap_start =
        std::find_if(s.x.begin(), s.x.end(), [right_snake_end](double x) { return x >= right_snake_end; }) -
        s.x.begin();
    const auto right_snake_overlap_start =
        std::find_if(other.x.begin(), other.x.end(), [left_snake_end](double x) { return x <= left_snake_end; }) -
        other.x.begin();
    const auto overlap_len =
        std::min(s.x.size() - left_snake_overlap_start, other.x.size() - right_snake_overlap_start) + 1;
    const auto dx  = (left_snake_end - right_snake_end) / overlap_len;
    auto overlap_x = std::vector<double>();
    auto overlap_y = std::vector<double>();

    for (int i = 0; i < overlap_len; i++) {
      const auto x = right_snake_end + dx * i;
      const auto f = static_cast<double>(i) / overlap_len;
      const auto first_left =
          std::find_if(s.x.begin() + left_snake_overlap_start, s.x.end(), [x](double left_x) { return left_x >= x; }) -
          s.x.begin();
      const auto first_right = std::find_if(other.x.begin() + right_snake_overlap_start, other.x.end(),
                                            [x](double right_x) { return right_x <= x; }) -
                               other.x.begin();
      const auto y1 = s.y[first_left];
      const auto y2 = other.y[first_right];
      overlap_x.push_back(x);
      overlap_y.push_back(y1 * (1.0 - f) + y2 * f);
    }

    auto new_overlap_y = std::vector<double>(overlap_y);

    if (overlap_len > 4) {
      // Smoothen the overlap to get better angles.
      for (int i = 2; i < overlap_len - 2; i++) {
        auto [c0, c1] = boost::math::statistics::simple_ordinary_least_squares(
            std::vector<double>(overlap_x.begin() + i - 2, overlap_x.begin() + i + 3),
            std::vector<double>(overlap_y.begin() + i - 2, overlap_y.begin() + i + 3));
        new_overlap_y[i] = c0 + c1 * overlap_x[i];
      }
    }

    s.x.erase(s.x.begin() + left_snake_overlap_start, s.x.end());
    s.y.erase(s.y.begin() + left_snake_overlap_start, s.y.end());
    other.x.erase(other.x.begin() + right_snake_overlap_start, other.x.end());
    other.y.erase(other.y.begin() + right_snake_overlap_start, other.y.end());
    s.x.insert(s.x.end(), overlap_x.begin(), overlap_x.end());
    s.y.insert(s.y.end(), new_overlap_y.begin(), new_overlap_y.end());
  }

  s.x.insert(s.x.end(), std::reverse_iterator(other.x.end()), std::reverse_iterator(other.x.begin()));
  s.y.insert(s.y.end(), std::reverse_iterator(other.y.end()), std::reverse_iterator(other.y.begin()));
  return s;
}

auto Snake::ToLPCS(core::image::CameraModel* camera, int vertical_offset, int horizontal_offset) const
    -> std::optional<std::tuple<core::image::WorkspaceCoordinates, uint8_t, uint8_t>> {
  auto c   = PlaneCoordinates(2, x.size());
  c.row(0) = Eigen::Map<const Eigen::RowVectorXd, Eigen::Unaligned>(x.data(), x.size()).array() + horizontal_offset;
  c.row(1) = Eigen::Map<const Eigen::RowVectorXd, Eigen::Unaligned>(y.data(), y.size());

  auto maybe_lpcs = camera->ImageToWorkspace(c, vertical_offset);
  if (maybe_lpcs.has_value()) {
    return {
        {maybe_lpcs.value(), min_pixel_value, max_pixel_value}
    };
  } else {
    return std::nullopt;
  }
};
