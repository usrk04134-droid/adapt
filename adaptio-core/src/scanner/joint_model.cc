#include "core/scanner/joint_model.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <Eigen/Core>
#include <string>
#include <vector>

#include "core/image/image_types.h"  // IWYU pragma: keep

using core::scanner::JointModel;
using core::scanner::JointModelErrorCode;
using core::scanner::LineSegment;

const double LASER_INCIDENCE_ANGLE = 0.1745329252;  // 10 degrees
auto JointModel::FitPoints(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& x_and_y,
                           double residual_threshold) -> core::scanner::LineSegment {
  return FitPoints(x_and_y.row(0), x_and_y.row(1), residual_threshold);
}

auto JointModel::FitPoints(const Eigen::RowVectorXd& x, const Eigen::RowVectorXd& y, double residual_threshold)
    -> core::scanner::LineSegment {
  using Eigen::Index;
  using pcl::PointCloud;
  using pcl::PointXYZ;
  using pcl::RandomSampleConsensus;
  using pcl::SampleConsensusModelLine;

  assert(x.size() == y.size());

  PointCloud<PointXYZ>::Ptr const cloud(new PointCloud<PointXYZ>);
  cloud->width    = x.cols();
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize(static_cast<size_t>(cloud->width) * cloud->height);

  for (Index i = 0; i < x.cols(); i++) {
    (*cloud)[i].x = static_cast<float>(x[i]);
    (*cloud)[i].y = static_cast<float>(y[i]);
    (*cloud)[i].z = 0.0F;
  }

  std::vector<int> inliers;
  Eigen::VectorXf coefficients;

  SampleConsensusModelLine<PointXYZ>::Ptr const line_model(new SampleConsensusModelLine<PointXYZ>(cloud));

  RandomSampleConsensus<PointXYZ> ransac(line_model);
  ransac.setDistanceThreshold(residual_threshold);
  ransac.computeModel();
  ransac.getInliers(inliers);
  ransac.getModelCoefficients(coefficients);

  LineSegment line = LineSegment(inliers.size());

  {
    for (int index = 0; int const inlier : inliers) {
      line.inliers.x(index) = cloud->points[inlier].x;
      line.inliers.y(index) = cloud->points[inlier].y;
      index++;
    }
    line.x_limits.min = line.inliers.x.minCoeff();
    line.x_limits.max = line.inliers.x.maxCoeff();
    line.y_limits.min = line.inliers.y.minCoeff();
    line.y_limits.max = line.inliers.y.maxCoeff();
  }

  line.inliers_indices = inliers;

  /* The line model coefficients are defined as:
   *   [0] : the X coordinate of a point on the line
   *   [1] : the Y coordinate of a point on the line
   *   [2] : the Z coordinate of a point on the line
   *   [3] : the X coordinate of the line's direction
   *   [4] : the Y coordinate of the line's direction
   *   [5] : the Z coordinate of the line's direction
   */
  // LOG_TRACE("Fitted model coefficients: {},{},{}, {},{},{}", coefficients[0], coefficients[1], coefficients[2],
  //           coefficients[3], coefficients[4], coefficients[5]);
  line.k     = coefficients[4] / coefficients[3];
  line.m     = coefficients[1] - line.k * coefficients[0];
  line.theta = std::atan(coefficients[4] / coefficients[3]);  // / (2 * M_PI) * 360.0;

  return line;
}

auto JointModel::LPCSToWeldObjectAngle(double angle) -> double {
  auto scaled_y = std::cos(angle) * std::cos(LASER_INCIDENCE_ANGLE);
  return std::atan(std::sin(angle) / scaled_y);
}

auto JointModel::LPCSFromWeldObjectAngle(double angle) -> double {
  return std::atan(std::tan(angle) * std::cos(LASER_INCIDENCE_ANGLE));
}

namespace core::scanner {
auto JointModelErrorCodeToString(JointModelErrorCode error_code) -> std::string {
  switch (error_code) {
    case JointModelErrorCode::NO_ERROR:
      return "No error";
    case JointModelErrorCode::SURFACE_NOT_FOUND:
      return "Surface not found";
    case JointModelErrorCode::WEDGE_FIT_FAILED:
      return "PWL wedge could not be constructed";
    case JointModelErrorCode::GROOVE_BOTTOM_NOT_FOUND:
      return "Groove bottom not found";
    case JointModelErrorCode::GROOVE_WALL_CENTROIDS_NOT_FOUND:
      return "Groove wall centroids not found";
    case JointModelErrorCode::MISSING_WEDGE_HISTORY:
      return "Cap welding mode requires a wedge history";
    case JointModelErrorCode::INVALID_SNAKE:
      return "The snake appears to have an invalid shape. Recommend increasing the threshold in the scanner "
             "configuration.";
    case JointModelErrorCode::INVALID_WALL_HEIGHT_DIFFERENCE:
      return "Difference between wall heights was considered unreasonable.";
    case JointModelErrorCode::SURFACE_ANGLE_TOLERANCE_EXCEEDED:
      return "The surface angle tolerances were exceeded.";
    case JointModelErrorCode::JOINT_WIDTH_OUT_OF_TOLERANCE:
      return "The joint width was out of tolerance.";
    case JointModelErrorCode::TWO_WALLS_NOT_FOUND:
      return "Two walls not found.";
    case JointModelErrorCode::FAULTY_HISTORY_DATA:
      return "Faulty history data.";
  }
}

auto JointModelErrorCodeToSnakeCaseString(JointModelErrorCode error_code) -> std::string {
  switch (error_code) {
    case JointModelErrorCode::NO_ERROR:
      return "no_error";
    case JointModelErrorCode::SURFACE_NOT_FOUND:
      return "surface_not_found";
    case JointModelErrorCode::WEDGE_FIT_FAILED:
      return "wedge_fit_failed";
    case JointModelErrorCode::GROOVE_BOTTOM_NOT_FOUND:
      return "groove_bottom_not_found";
    case JointModelErrorCode::GROOVE_WALL_CENTROIDS_NOT_FOUND:
      return "groove_wall_centroids_not_found";
    case JointModelErrorCode::MISSING_WEDGE_HISTORY:
      return "missing_wedge_history";
    case JointModelErrorCode::INVALID_SNAKE:
      return "invalid_snake";
    case JointModelErrorCode::INVALID_WALL_HEIGHT_DIFFERENCE:
      return "invalid_wall_height_difference";
    case JointModelErrorCode::SURFACE_ANGLE_TOLERANCE_EXCEEDED:
      return "surface_angle_tolerance_exceeded";
    case JointModelErrorCode::JOINT_WIDTH_OUT_OF_TOLERANCE:
      return "joint_width_out_of_tolerance";
    case JointModelErrorCode::TWO_WALLS_NOT_FOUND:
      return "two_walls_not_found";
    case JointModelErrorCode::FAULTY_HISTORY_DATA:
      return "faulty_history_data";  }
}
}  // namespace core::scanner

#ifndef DOCTEST_CONFIG_DISABLE
// NOLINTBEGIN(*-magic-numbers)

#include <doctest/doctest.h>

using Eigen::Index;
using Eigen::Matrix3d;
using Eigen::RowVectorX;
using Eigen::RowVectorXd;
using Eigen::Vector3d;

using core::image::CameraProperties;
using core::image::RawImageData;
using core::scanner::JointProperties;

class JointModelExposed : public JointModel {
 public:
  using core::scanner::JointModel::FitPoints;
};

TEST_SUITE("Joint Model") {
  TEST_CASE("Simple point fitting") {
    RowVectorXd x(5);
    x << 0, 3, 6, 9, 12;

    RowVectorXd y(5);
    y << 2, 4, 6, 8, 10;

    auto line_model = JointModelExposed::FitPoints(x, y, 0.03);

    CHECK_EQ(line_model.k, 2.0f / 3.0f);
    CHECK_EQ(line_model.m, 2.0f);
    CHECK_EQ(line_model.inliers.x.cols(), 5);
    CHECK_EQ(line_model.inliers.y.cols(), 5);
  }

  TEST_CASE("Simple point fitting 45 degrees") {
    RowVectorXd x(5);
    x << 0, 3, 6, 9, 12;

    RowVectorXd y(5);
    y << 2, 5, 8, 11, 14;

    auto line_model = JointModelExposed::FitPoints(x, y, 0.03);

    CHECK_EQ(line_model.k, 1.0f);
    CHECK_EQ(line_model.m, 2.0f);
    CHECK_LT(std::abs(line_model.theta - 45.0 * static_cast<double>(EIGEN_PI) / 180.0), 0.0001);
    CHECK_EQ(line_model.inliers.x.cols(), 5);
    CHECK_EQ(line_model.inliers.y.cols(), 5);
  }

  TEST_CASE("Simple point fitting with outliers") {
    RowVectorXd x(5);
    x << 0, 3, 6, 9, 12;

    RowVectorXd y(5);
    y << 2, 4, 42, 8, 100;

    auto line_model = JointModelExposed::FitPoints(x, y, 0.03);

    CHECK_EQ(line_model.k, 2.0f / 3.0f);
    CHECK_EQ(line_model.m, 2.0f);
    CHECK_EQ(line_model.inliers.x.cols(), 3);
    CHECK_EQ(line_model.inliers.y.cols(), 3);
    CHECK_EQ(line_model.inliers.x(0), 0);
    CHECK_EQ(line_model.inliers.y(0), 2);
    CHECK_EQ(line_model.inliers.x(1), 3);
    CHECK_EQ(line_model.inliers.y(1), 4);
    CHECK_EQ(line_model.inliers.x(2), 9);
    CHECK_EQ(line_model.inliers.y(2), 8);
  }

  TEST_CASE("Point fitting with outliers") {
    RowVectorXd x(5);
    x << 3, 6, 9, 12, 15;

    RowVectorXd y(5);
    y << 3.9, 6.1, 7.8, 10.2, 120;

    auto line_model = JointModelExposed::FitPoints(x, y, 0.3);

    CHECK_EQ(line_model.inliers.x.cols(), 4);
    CHECK_EQ(line_model.inliers.y.cols(), 4);

    CHECK_LT(line_model.k - 0.7f, 0.0001);
    CHECK_LT(line_model.m - 1.8f, 0.0001);
    CHECK_LT(line_model.inliers.x(0) - 3, 0.0001);
    CHECK_LT(line_model.inliers.y(0) - 3.9, 0.0001);
    CHECK_LT(line_model.inliers.x(1) - 6, 0.0001);
    CHECK_LT(line_model.inliers.y(1) - 6.2, 0.0001);
    CHECK_LT(line_model.inliers.x(2) - 9, 0.0001);
    CHECK_LT(line_model.inliers.y(2) - 7.8, 0.0001);
    CHECK_LT(line_model.inliers.x(3) - 12, 0.0001);
    CHECK_LT(line_model.inliers.y(3) - 10.2, 0.0001);
  }
}

// NOLINTEND(*-magic-numbers)
#endif
