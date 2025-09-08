#include "scanner/joint_model/joint_model.h"

#include <doctest/doctest.h>

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <Eigen/Core>

#include "scanner/image/image_types.h"  // IWYU pragma: keep

using Eigen::Index;
using Eigen::Matrix3d;
using Eigen::RowVectorX;
using Eigen::RowVectorXd;
using Eigen::Vector3d;

// NOLINTBEGIN(*-magic-numbers)
namespace scanner::joint_model {

class JointModelExposed : public JointModel {
 public:
  using JointModel::FitPoints;
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
}  // namespace scanner::joint_model
// NOLINTEND(*-magic-numbers)
