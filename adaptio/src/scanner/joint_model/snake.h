#pragma once

#include <boost/circular_buffer.hpp>
#include <Eigen/Eigen>
#include <expected>
#include <optional>
#include <tuple>
#include <vector>

#include "scanner/image/camera_model.h"
#include "scanner/image/image.h"
#include "scanner/image/image_types.h"
#include "scanner/joint_model/joint_model.h"

namespace scanner::joint_model {

enum class StartingPoint { Left, Right };

struct Snake {
  std::vector<double> x;
  std::vector<double> y;

  uint8_t min_pixel_value;
  uint8_t max_pixel_value;

  /**
   * Finds the "snake", a contiguous list of coordinates starting from either
   * the left or the right side.
   *
   * @param image            Image
   * @param mask             A mask for the image, indicating whether to use this pixel or not.
   * @param threshold        Black level
   * @return The snake (a Snake instance), or nullopt on failure.
   */
  static auto FromImage(const image::Image& image, const std::optional<image::RawImageData>& mask, uint8_t threshold)
      -> std::optional<Snake>;

  /**
   * Convert to Eigen matrix in LPCS coordinates
   *
   * @param camera          The camera model
   * @param vertical_offset The crop start used for this image.
   * @return 3xN Eigen matrix with LPCS coordinates, or nullopt on failure.
   */
  auto ToLPCS(image::CameraModel* camera, int vertical_offset, int horizontal_offset) const
      -> std::optional<std::tuple<image::WorkspaceCoordinates, uint8_t, uint8_t>>;

 protected:
  /**
   * Finds the "snake", a contiguous list of coordinates starting from either
   * the left or the right side.
   *
   * @param image            Image
   * @param mask             A mask for the image, indicating whether to use this pixel or not.
   * @param side             The side to start from (left or right)
   * @param threshold        Black level
   * @param horizontal_limit When to stop
   * @return The snake (a Snake instance), or nullopt on failure.
   */
  static auto FromImage(const image::Image& image, const std::optional<image::RawImageData>& mask, uint8_t threshold,
                        enum StartingPoint side, double horizontal_limit) -> std::optional<Snake>;

  /**
   * Find the leftmost or rightmost column with at least one pixel
   * above the threshold.
   *
   * @param image     Image
   * @param threshold Black level
   * @param side      The side to start from (left or right)
   * @return The first column where this condition is true, or nullopt if the entire image is below the threshold.
   */
  static auto FindStartColumn(const image::Image& image, uint8_t threshold, enum StartingPoint side)
      -> std::optional<int>;

  /**
   * Merge (optionally) two snakes, cutting out parts of snakes that terminate prematurely.
   * If the snakes don't meet in the middle merging fails.
   *
   * @param left  The left snake
   * @param right The right snake
   * @return The joined snake, or nullopt on failure.
   */
  static auto Merge(const std::optional<Snake>& left, const std::optional<Snake>& right) -> std::optional<Snake>;

  /**
   * Reverse a snake
   *
   * @return The snake but in reverse
   */
  auto Reversed() const -> Snake;

  /**
   * Appends a second snake (but in reverse) to this snake.
   * If there is overlap in x, cut out the overlapping parts and
   * replace with an average linear interpolation.
   *
   * @param other The other snake
   * @return The joined snake.
   */
  auto ConcatenateInReverse(const Snake other) const -> Snake;
};

}  // namespace scanner::joint_model
