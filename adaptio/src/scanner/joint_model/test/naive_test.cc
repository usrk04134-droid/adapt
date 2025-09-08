#include "scanner/joint_model/naive.h"

#include <doctest/doctest.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Core/util/IndexedViewHelper.h>
#include <math.h>

#include <array>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <opencv2/core/eigen.hpp>
#include <optional>
#include <ranges>
#include <tuple>
#include <utility>
#include <vector>

#include "common/logging/application_log.h"
#include "scanner/image/camera_model.h"
#include "scanner/joint_model/joint_model.h"
#include "scanner/scanner_configuration.h"
#include "test_data.h"

using Eigen::Index;
using Eigen::Matrix3d;
using Eigen::RowVectorX;
using Eigen::RowVectorXd;
using Eigen::Vector3d;

// NOLINTBEGIN(*-magic-numbers)
namespace scanner::joint_model {

/**
 * This function is used to bootstrap tests. It generates a point cloud
 * vaguely resembling a laser line.
 * @param image_width
 * @param image_height
 * @param width
 * @param height
 * @param surface_height
 * @param left_surface_angle
 * @param right_surface_angle
 * @param groove_width
 * @param groove_angle
 * @return
 */
static auto SetupTest(int image_width, int image_height, double width, double height, double surface_height,
                      double left_surface_angle, double right_surface_angle, double groove_width, double groove_angle)
    -> std::tuple<RowVectorXd, RowVectorXd, RowVectorXd, RowVectorXd, RowVectorXd, RowVectorXd> {
  const double left_surface_angle_radians  = left_surface_angle;
  const double right_surface_angle_radians = right_surface_angle;
  const double groove_angle_radians        = groove_angle;

  RowVectorXd x = RowVectorXd::LinSpaced(image_width, 0.0, width);
  RowVectorXd y(image_width);

  LOG_TRACE("Start: {}, {}", 0.0, surface_height);

  // Left surface
  double left_corner_x = NAN;
  double left_corner_y = NAN;
  Index left_corner_index;
  for (Index i = 0; i < image_width; i++) {
    if (x[i] <= (width - groove_width) / 2.0) {
      y[i]              = surface_height + tan(left_surface_angle_radians) * x[i];
      left_corner_x     = x[i];
      left_corner_y     = y[i];
      left_corner_index = i;
    }
  }

  LOG_TRACE("Left corner: {}, {}", left_corner_x, left_corner_y);

  // Left groove line
  double groove_bottom_x = NAN;
  double groove_bottom_y = NAN;
  for (Index i = 0; i < image_width; i++) {
    if (x[i] >= left_corner_x && x[i] <= left_corner_x + groove_width / 2.0) {
      y[i]            = left_corner_y - tan(groove_angle_radians / 2.0) * (x[i] - left_corner_x);
      groove_bottom_x = x[i];
      groove_bottom_y = y[i];
    }
  }

  LOG_TRACE("Mid: {}, {}", groove_bottom_x, groove_bottom_y);

  // Right groove line
  double groove_right_x = NAN;
  double groove_right_y = NAN;
  Index right_corner_index;
  for (Index i = 0; i < image_width; i++) {
    if (x[i] >= groove_bottom_x && x[i] <= left_corner_x + groove_width) {
      y[i]               = groove_bottom_y + tan(groove_angle_radians / 2.0) * (x[i] - groove_bottom_x);
      groove_right_x     = x[i];
      groove_right_y     = y[i];
      right_corner_index = i;
    }
  }

  LOG_TRACE("Right corner: {}, {}", groove_right_x, groove_right_y);

  // Right surface
  double end_x = NAN;
  double end_y = NAN;
  for (Index i = 0; i < image_width; i++) {
    if (x[i] >= groove_right_x) {
      y[i]  = groove_right_y + tan(right_surface_angle_radians) * (x[i] - groove_right_x);
      end_x = x[i];
      end_y = y[i];
    }
  }

  LOG_TRACE("End: {}, {}", end_x, end_y);

  // Extract groove
  RowVectorXd left_wall_x  = x.block(0, left_corner_index, 1, image_width / 2.0 - left_corner_index);
  RowVectorXd left_wall_y  = y.block(0, left_corner_index, 1, image_width / 2.0 - left_corner_index);
  RowVectorXd right_wall_x = x.block(0, x.cols() / 2, 1, right_corner_index - x.cols() / 2);
  RowVectorXd right_wall_y = y.block(0, y.cols() / 2, 1, right_corner_index - y.cols() / 2);

  return {x, y, left_wall_x, left_wall_y, right_wall_x, right_wall_y};
}

class NaiveExposed : public Naive {
 public:
  explicit NaiveExposed(const JointProperties& properties, const scanner::ScannerConfigurationData& scanner_config)
      : Naive(properties, scanner_config, image::CameraModelPtr{}) {};
  using joint_model::Naive::FitGrooveLine;
  using joint_model::Naive::GetImprovedSurfaceLine;
  using joint_model::Naive::GetMedianGrooveDepths;
  using joint_model::Naive::groove_wall_height_buffer_;
  using joint_model::Naive::GrooveBottomLine;
  using joint_model::Naive::GrooveLine;
  using joint_model::Naive::SurfaceLines;
};

TEST_SUITE("Naive Joint Model") {
  TEST_CASE("Surface line find") {
    const int image_width            = 3500;
    const int image_height           = 2000;
    const double width               = 120.0 * 1.0e-3;
    const double height              = image_height / static_cast<double>(image_width) * width;
    const double surface_height      = 30.0 * 1.0e-3;
    const double left_surface_angle  = 10.0 * static_cast<double>(EIGEN_PI) / 180.0;
    const double right_surface_angle = 4.0 * static_cast<double>(EIGEN_PI) / 180.0;
    const double groove_width        = 40.0 * 1.0e-3;
    const double groove_angle        = 60.0 * static_cast<double>(EIGEN_PI) / 180.0;

    auto joints = SetupTest(image_width, image_height, width, height, surface_height, left_surface_angle,
                            right_surface_angle, groove_width, groove_angle);

    auto x = std::get<0>(joints);
    auto y = std::get<1>(joints);

    JointProperties joint_properties = {};

    joint_properties.surface_angle_tolerance = 10.0 * static_cast<double>(EIGEN_PI) / 180.0;
    joint_properties.left_max_surface_angle  = left_surface_angle + 0.1 * static_cast<double>(EIGEN_PI) / 180.0;
    joint_properties.right_max_surface_angle = right_surface_angle + 0.1 * static_cast<double>(EIGEN_PI) / 180.0;
    joint_properties.upper_joint_width       = groove_width * 1.0e3;

    NaiveExposed exposed(joint_properties, {16, 16, 16});
    auto [left, right] = exposed.SurfaceLines(x, y);

    if (left.has_value()) {
      LOG_TRACE("{}", left.value().ToString());
      CHECK_LE(abs(left.value().theta - left_surface_angle), 0.001);
    } else {
      CHECK(false);
    }

    if (right.has_value()) {
      LOG_TRACE("{}", right.value().ToString());
      CHECK_LE(abs(right.value().theta - right_surface_angle), 0.001);
    } else {
      CHECK(false);
    }
  }

  TEST_CASE("Groove line find") {
    using Eigen::Matrix;

    const int image_width            = 3500;
    const int image_height           = 2000;
    const double width               = 120.0 * 1.0e-3;
    const double height              = image_height / static_cast<double>(image_width) * width;
    const double surface_height      = 30.0 * 1.0e-3;
    const double left_surface_angle  = 10.0 * static_cast<double>(EIGEN_PI) / 180.0;
    const double right_surface_angle = 4.0 * static_cast<double>(EIGEN_PI) / 180.0;
    const double groove_width        = 40.0 * 1.0e-3;
    const double groove_angle        = 60.0 * static_cast<double>(EIGEN_PI) / 180.0;

    auto joints = SetupTest(image_width, image_height, width, height, surface_height, left_surface_angle,
                            right_surface_angle, groove_width, groove_angle);

    auto left_wall_x  = std::get<2>(joints);
    auto left_wall_y  = std::get<3>(joints);
    auto right_wall_x = std::get<4>(joints);
    auto right_wall_y = std::get<5>(joints);

    JointProperties joint_properties = {};

    joint_properties.upper_joint_width           = groove_width * 1.0e3;
    joint_properties.upper_joint_width_tolerance = 1.0;

    Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor> left_centroids(3, left_wall_x.cols());
    left_centroids.block(0, 0, 1, left_wall_x.cols()) = left_wall_x;
    left_centroids.block(1, 0, 1, left_wall_x.cols()) = left_wall_y;

    NaiveExposed exposed(joint_properties, {16, 16, 16});
    auto maybe_left_groove_line = exposed.GrooveLine(left_centroids);

    CHECK(maybe_left_groove_line.has_value());

    CHECK_LE(abs(maybe_left_groove_line.value().theta + groove_angle / 2.0), 0.001);
  }

  TEST_CASE("Fit wedge") {
    JointProperties joint_properties = {};

    joint_properties.upper_joint_width           = 50.0 * 1e3;
    joint_properties.left_joint_angle            = 30.0 * static_cast<double>(EIGEN_PI) / 180.0;
    joint_properties.right_joint_angle           = 30.0 * static_cast<double>(EIGEN_PI) / 180.0;
    joint_properties.upper_joint_width_tolerance = 5.0 * 1e3;
    joint_properties.groove_angle_tolerance      = 5.0 * static_cast<double>(EIGEN_PI) / 180.0;

    NaiveExposed exposed(joint_properties, {16, 16, 16});

    LineSegment left_top_surface(100);
    left_top_surface.k = 2.4188305251300335e-03;
    left_top_surface.m = 1.0309065133333206e-01;
    LineSegment right_top_surface(100);
    right_top_surface.k = -1.5275700949132442e-02;
    right_top_surface.m = 1.0255440324544907e-01;

    std::tuple<double, double, double> left_surface_edge  = {-3.9591481451023654e-02, 1.0299610130067234e-01, 0.0};
    std::tuple<double, double, double> right_surface_edge = {5.7430421588993263e-03, 1.0247409568516797e-01, 0.0};

    LineSegment left_wall(left_wall_x.size());
    LineSegment right_wall(right_wall_x.size());

    left_wall.inliers.x  = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(left_wall_x.data(), left_wall_x.size());
    left_wall.inliers.y  = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(left_wall_y.data(), left_wall_y.size());
    right_wall.inliers.x = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(right_wall_x.data(), right_wall_x.size());
    right_wall.inliers.y = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(right_wall_y.data(), right_wall_y.size());

    left_wall.x_limits.min  = left_wall_x[0];
    left_wall.x_limits.max  = left_wall_x[left_wall_x.size() - 1];
    right_wall.x_limits.min = right_wall_x[0];
    right_wall.x_limits.max = right_wall_x[right_wall_x.size() - 1];
    left_wall.y_limits.min  = left_wall_y[0];
    left_wall.y_limits.max  = left_wall_y[left_wall_y.size() - 1];
    right_wall.y_limits.min = right_wall_y[0];
    right_wall.y_limits.max = right_wall_y[right_wall_y.size() - 1];
    left_wall.theta         = -1.05;
    right_wall.theta        = 1.05;

    auto maybe_pwl_wedge = exposed.FitGrooveLine(left_top_surface, right_top_surface, left_wall, right_wall,
                                                 left_surface_edge, right_surface_edge);
    if (maybe_pwl_wedge.has_value()) {
      auto pwl_wedge = maybe_pwl_wedge.value();
      CHECK_LE(abs(pwl_wedge.y0t - 10.3091), 0.0001);
      // TODO: Estimate estimate k+m values for left_wall and right_wall so that x1,y1,x2,y2,x3,y3 will be defined.
      // CHECK_LE(abs(pwl_wedge.x1t - (-3.96195963)), 0.0001);
      // CHECK_LE(abs(pwl_wedge.y1t - 10.2995), 0.0001);
      // CHECK_LE(abs(pwl_wedge.x2t - (-1.91182607)), 0.0001);
      // CHECK_LE(abs(pwl_wedge.y2t - 6.58339778), 0.0001);
      // CHECK_LE(abs(pwl_wedge.x3t - 0.58522478), 0.0001);
      // CHECK_LE(abs(pwl_wedge.y3t - 10.24650061), 0.0001);
      CHECK_LE(abs(pwl_wedge.y4t - 10.2554), 0.0001);
    } else {
      CHECK(false);
    }
  }

  TEST_CASE("Groove bottom") {
    JointProperties joint_properties = {};

    joint_properties.upper_joint_width           = 45.0;
    joint_properties.left_joint_angle            = 30.0 * static_cast<double>(EIGEN_PI) / 180.0;
    joint_properties.right_joint_angle           = 30.0 * static_cast<double>(EIGEN_PI) / 180.0;
    joint_properties.upper_joint_width_tolerance = 5.0;
    joint_properties.groove_angle_tolerance      = 5.0 * static_cast<double>(EIGEN_PI) / 180.0;
    joint_properties.groove_depth                = 60.0;

    NaiveExposed exposed(joint_properties, {16, 16, 16});

    Eigen::RowVectorXd x_coords(lpcs_x_coords.size());
    Eigen::RowVectorXd z_coords(lpcs_z_coords.size());

    x_coords = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(lpcs_x_coords.data(), lpcs_x_coords.size());
    z_coords = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(lpcs_z_coords.data(), lpcs_z_coords.size());

    LineSegment left_wall(left_wall_x.size());
    LineSegment right_wall(right_wall_x.size());

    left_wall.inliers.x  = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(left_wall_x.data(), left_wall_x.size());
    left_wall.inliers.y  = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(left_wall_y.data(), left_wall_y.size());
    right_wall.inliers.x = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(right_wall_x.data(), right_wall_x.size());
    right_wall.inliers.y = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(right_wall_y.data(), right_wall_y.size());

    left_wall.x_limits.max  = -0.021572690357988252;
    right_wall.x_limits.min = -0.01575632277317308;
    left_wall.y_limits.min  = 7.0282891392707825e-02;
    right_wall.y_limits.min = 7.0282891392707825e-02;

    PwlWedge pwl_wedge{
        .y0t     = 1.0299733308498569e+01,
        .x1t     = -3.9619596192279607e+00,
        .y1t     = 1.0299481824466884e+01,
        .x2t     = -1.9118260710987687e+00,
        .y2t     = 6.5833977763209042e+00,
        .x3t     = 5.8522477515062754e-01,
        .y3t     = 1.0246500605891683e+01,
        .y4t     = 1.0245704293258299e+01,
        .x_left  = -2.1572690418818225e-02,
        .x_right = -1.5756322752176505e-02,
    };

    exposed.GrooveBottomLine(x_coords, z_coords, pwl_wedge);
  }
}

}  // namespace scanner::joint_model
// NOLINTEND(*-magic-numbers)
