#pragma once

#include <Eigen/Eigen>

#include "core/logging/application_log.h"

namespace core::math {

const unsigned int MINIMUM_NUMBER_OF_DEFINED_CENTROIDS = 15;
const int COG_WIDTH = 13;  // The number of pixels around the maximum to look for the center of gravity.

enum CentroidSearchDirection { Normal, Reversed };

static auto GetCentroidForSingleColumn(const Eigen::Block<const Eigen::Matrix<unsigned char, -1, -1, 1>, -1, 1>& column,
                                       CentroidSearchDirection direction, uint8_t threshold) -> std::optional<double> {
  using Eigen::Index;
  using Eigen::RowVectorX;

  const int half = (COG_WIDTH - 1) / 2;

  Index index_for_max = 0;
  uint8_t max_value;
  if (direction == CentroidSearchDirection::Reversed) {
    max_value     = column.reverse().maxCoeff(&index_for_max);
    index_for_max = column.rows() - index_for_max - 1;
  } else {
    max_value = column.maxCoeff(&index_for_max);
  }

  if (max_value == 0xff) {
    // If the maximum value is a saturated pixel, then we take the median of y values for all saturated pixels
    std::vector<Index> y_values;
    for (int i = 0; i < column.rows(); i++) {
      if (column[i] == 0xff) {
        y_values.push_back(i);
      }
    }
    if (y_values.size() > 1) {
      std::nth_element(y_values.begin(), y_values.begin() + y_values.size() / 2, y_values.end());
      index_for_max = y_values[y_values.size() / 2];
    }
  }

  if (max_value >= threshold) {
    Index top, bottom;
    if (index_for_max + half >= column.rows()) {
      bottom = column.rows() - 1;
    } else {
      bottom = index_for_max + half;
    }
    if (index_for_max >= half) {
      top = index_for_max - half;
    } else {
      top = 0;
    }

    const auto rows         = RowVectorX<double>::LinSpaced(bottom - top + 1, 0.0, static_cast<double>(bottom - top));
    const auto columns      = column.segment(top, bottom - top + 1).cast<double>();
    const double total_mass = columns.sum();

    if (total_mass > 0.0) {
      return columns.dot(rows) / total_mass + static_cast<double>(top);
    }
  }
  return std::nullopt;
}

/**
 * Calculates the centroids around the maximum of each column for the given matrix.
 *
 * First locate the maximum (starting from the bottom) for each column.
 * Then calculate the centroid in a COG_WIDTH tall region around this maximum.
 *
 * @param matrix The matrix
 * @return A matrix consisting of two row vectors, where the first is the column index
 * (x) and the second the row index (y) where the columnar centroid is located
 * in the original matrix. Any zero columns will be removed from
 * consideration.
 */
[[maybe_unused]] static auto GetCentroidsByColumn(
    const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& matrix,
    CentroidSearchDirection direction, uint8_t threshold)
    -> std::optional<Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::RowMajor>> {
  using Eigen::Index;
  using Eigen::Matrix;
  using Eigen::RowVectorX;

  Matrix<double, 2, Eigen::Dynamic, Eigen::RowMajor> centroids(2, matrix.cols());

  Index number_of_defined = 0;
  Index index             = 0;

  for (auto column : matrix.colwise()) {
    auto maybe_centroid = GetCentroidForSingleColumn(column, direction, threshold);

    if (maybe_centroid.has_value()) {
      const double column_index       = static_cast<double>(index);
      centroids(0, number_of_defined) = column_index;
      centroids(1, number_of_defined) = maybe_centroid.value();
      number_of_defined++;
    }

    index++;
  }

  LOG_TRACE("Number of defined centroids = {}", number_of_defined);

  if (number_of_defined < std::min(static_cast<unsigned int>(matrix.cols()), MINIMUM_NUMBER_OF_DEFINED_CENTROIDS)) {
    return std::nullopt;
  }

  LOG_TRACE("Centroids first: {},{}", centroids(0, 0), centroids(1, 0));
  LOG_TRACE("Centroids last: {},{}", centroids(0, number_of_defined - 1), centroids(1, number_of_defined - 1));

  return std::optional<Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::RowMajor>>{
      centroids.block(0, 0, 2, number_of_defined)};
}

/**
 * Calculates the centroids around the maximum of each row for the given matrix.
 *
 * First locate the maximum for each row.
 * Then calculate the centroid in a COG_WIDTH wide region around this maximum.
 *
 * @param matrix The matrix
 * @return A matrix consisting of two row vectors, where the first is the column index
 * (x) and the second the row index (y) where the row-wise centroid is located
 * in the original matrix. Any zero columns will be removed from
 * consideration.
 */
[[maybe_unused]] static auto GetCentroidsByRow(
    const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& matrix,
    CentroidSearchDirection direction, uint8_t threshold)
    -> std::optional<Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::RowMajor>> {
  using Eigen::Index;
  using Eigen::Matrix;
  using Eigen::RowVectorX;

  Matrix<double, 2, Eigen::Dynamic, Eigen::RowMajor> centroids(2, matrix.rows());

  Index number_of_defined = 0;
  Index index             = 0;
  const int half          = (COG_WIDTH - 1) / 2;

  for (auto row : matrix.rowwise()) {
    Index index_for_max;
    uint8_t max_value;
    // // This implementation is shorter but significantly slower
    if (direction == CentroidSearchDirection::Reversed) {
      max_value     = row.reverse().maxCoeff(&index_for_max);
      index_for_max = matrix.cols() - index_for_max - 1;
    } else {
      max_value = row.maxCoeff(&index_for_max);
    }
    if (max_value >= threshold) {
      Index left, right;
      if (index_for_max + half >= matrix.cols()) {
        right = matrix.cols() - 1;
      } else {
        right = index_for_max + half;
      }
      if (index_for_max >= half) {
        left = index_for_max - half;
      } else {
        left = 0;
      }

      auto columns            = RowVectorX<double>::LinSpaced(right - left + 1, 0.0, static_cast<double>(right - left));
      auto rows               = row.segment(left, right - left + 1).cast<double>();
      const double total_mass = rows.sum();

      if (total_mass > 0.0) {
        const double centroid  = rows.dot(columns) / total_mass + static_cast<double>(left);
        const double row_index = static_cast<double>(index);

        centroids(0, number_of_defined) = centroid;
        centroids(1, number_of_defined) = row_index;
        number_of_defined++;
      }
    }

    index++;
  }

  LOG_TRACE("Number of defined wall centroids = {}", number_of_defined);

  if (number_of_defined < std::min(static_cast<unsigned int>(matrix.rows()), MINIMUM_NUMBER_OF_DEFINED_CENTROIDS)) {
    return std::nullopt;
  }

  return std::optional<Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::RowMajor>>{
      centroids.block(0, 0, 2, number_of_defined)};
}

}  // namespace core::math
