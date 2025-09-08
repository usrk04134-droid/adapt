#include <cmath>
#include <cstdlib>
#include <numbers>
#include <optional>
#include <tuple>

#include "scanner/image/camera_model.h"
#ifndef DOCTEST_CONFIG_DISABLE
#include <doctest/doctest.h>

#include "scanner/joint_model/joint_model.h"
#include "scanner/joint_model/slice.h"
#include "slice_test_data.h"

using Eigen::Index;
using Eigen::RowVectorXd;
using scanner::image::WorkspaceCoordinates;
using scanner::joint_model::JointProperties;
using scanner::joint_model::Point;
using scanner::joint_model::Slice;

TEST_SUITE("Test Slice") {
  const JointProperties properties{
      .upper_joint_width           = 40,
      .left_joint_angle            = 30 * 2 * std::numbers::pi / 360,
      .right_joint_angle           = 30 * 2 * std::numbers::pi / 360,
      .upper_joint_width_tolerance = 0.1,
      .surface_angle_tolerance     = 10 * 2 * std::numbers::pi / 360,
      .groove_angle_tolerance      = 10 * 2 * std::numbers::pi / 360,
      .offset_distance             = 3.0,
  };

  const JointProperties properties1{
      .upper_joint_width           = 30,
      .left_joint_angle            = 0.5235987755982988,
      .right_joint_angle           = 0.5235987755982988,
      .upper_joint_width_tolerance = 7.0,
      .surface_angle_tolerance     = 10 * 2 * std::numbers::pi / 360,
      .groove_angle_tolerance      = 10 * 2 * std::numbers::pi / 360,
      .offset_distance             = 3.0,
  };

  double fRand(double fMin, double fMax) {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
  }

  TEST_CASE("FromSnake - no walls") {
    // Snake is a straight line
    double x = 0.;
    WorkspaceCoordinates snake(3, 100);
    for (int i = 0; i < 100; i++) {
      snake.row(0)[i]  = x;
      x               += 1.e-3;
      // FindIntersect does not work with a complete straight line
      // add a small variation vertically. Look into this later
      snake.row(1)[i] = fRand(0., 0.1) * 1.e-3;
    }

    bool found_out_of_spec_joint_width = false;
    auto result =
        Slice::FromSnake(snake, properties, std::nullopt, found_out_of_spec_joint_width, false, false, std::nullopt);
    CHECK(!result);

    result =
        Slice::FromSnake(snake, properties, std::nullopt, found_out_of_spec_joint_width, false, true, std::nullopt);
    CHECK(!result);

    // Approximated abw0 and abw6 a bit lower than snake
    double abw0_approximation = 30 * 1e-3;
    double abw6_approximation = 70 * 1e-3;
    result = Slice::FromSnake(snake, properties, std::nullopt, found_out_of_spec_joint_width, false, true,
                              std::make_tuple(abw0_approximation, abw6_approximation));
    CHECK(result.has_value());
    auto [points, num_wall, approximation_used] = result.value();

    CHECK(num_wall == 0);
    CHECK(points[0].x == abw0_approximation);
    CHECK(std::fabs(points[0].y) < 0.0001);
    CHECK(std::fabs(points[6].x - abw6_approximation) < 0.000001);
    CHECK(std::fabs(points[6].y) < 0.0001);
  }
  TEST_CASE("FromSnake - faulty approximation data") {
    // Snake is a straight line
    double x = 0.;
    WorkspaceCoordinates snake(3, 100);
    for (int i = 0; i < 100; i++) {
      snake.row(0)[i]  = x;
      x               += 1.e-3;
      snake.row(1)[i]  = fRand(0., 0.1) * 1.e-3;
    }

    bool found_out_of_spec_joint_width = false;

    // Approximated abw6 outside of snake
    double abw0_approximation = 30 * 1e-3;
    double abw6_approximation = 110 * 1e-3;
    auto result = Slice::FromSnake(snake, properties, std::nullopt, found_out_of_spec_joint_width, false, true,
                                   std::make_tuple(abw0_approximation, abw6_approximation));
    CHECK(!result);
  }

  TEST_CASE("FromSnake - two walls") {
    // Better way to initialize a snake from vector?
    WorkspaceCoordinates snake(3, snake_two_walls.size() / 3);
    int index = 0;
    for (int i = 0; i < snake_two_walls.size() / 3; i++) {
      snake.row(0)[i]  = snake_two_walls[index];
      snake.row(1)[i]  = snake_two_walls[index + 1];
      snake.row(2)[i]  = snake_two_walls[index + 2];
      index           += 3;
    }
    bool found_out_of_spec_joint_width = false;

    auto result =
        Slice::FromSnake(snake, properties1, std::nullopt, found_out_of_spec_joint_width, false, false, std::nullopt);
    CHECK(result);
    auto abw_points         = get<0>(result.value());
    auto num_walls          = get<1>(result.value());
    auto approximation_used = get<2>(result.value());

    CHECK(num_walls == 2);
    CHECK(!approximation_used);
    CHECK(fabs(abw_points[0].x - 0.0610796) < 0.00001);
    CHECK(fabs(abw_points[0].y - -0.280483) < 0.00001);
    CHECK(fabs(abw_points[6].x - 0.0892176) < 0.00001);
    CHECK(fabs(abw_points[6].y - -0.28038) < 0.00001);

    CHECK(abw_points[0].x < abw_points[1].x);
    CHECK(abw_points[1].x < abw_points[2].x);
    CHECK(abw_points[2].x < abw_points[3].x);
    CHECK(abw_points[3].x < abw_points[4].x);
    CHECK(abw_points[4].x < abw_points[5].x);
    CHECK(abw_points[5].x < abw_points[6].x);

    // Use same abw0 aw6 as approximation points but move them 2mm horizontal
    auto abw0 = abw_points[0].x + 0.002;
    auto abw6 = abw_points[6].x + 0.002;

    std::tuple<double, double> abw0_abw6 = {abw0, abw6};
    result = Slice::FromSnake(snake, properties1, std::nullopt, found_out_of_spec_joint_width, false, true, abw0_abw6);
    CHECK(result);
    abw_points         = get<0>(result.value());
    num_walls          = get<1>(result.value());
    approximation_used = get<2>(result.value());

    CHECK(approximation_used);
    CHECK(num_walls == 0);
    CHECK(fabs(abw_points[0].x - (0.0610796 + 0.002)) < 0.00001);
    CHECK(fabs(abw_points[6].x - (0.0892176 + 0.002)) < 0.00001);

    CHECK(abw_points[0].x < abw_points[1].x);
    CHECK(abw_points[1].x < abw_points[2].x);
    CHECK(abw_points[2].x < abw_points[3].x);
    CHECK(abw_points[3].x < abw_points[4].x);
    CHECK(abw_points[4].x < abw_points[5].x);
    CHECK(abw_points[5].x < abw_points[6].x);
  }

  TEST_CASE("FromSnake - real snake no wall ") {
    // Better way to initialize a snake from vector?
    WorkspaceCoordinates snake(3, snake_no_walls.size() / 3);
    int index = 0;
    for (int i = 0; i < snake_no_walls.size() / 3; i++) {
      snake.row(0)[i]  = snake_no_walls[index];
      snake.row(1)[i]  = snake_no_walls[index + 1];
      snake.row(2)[i]  = snake_no_walls[index + 2];
      index           += 3;
    }
    bool found_out_of_spec_joint_width = false;

    auto result =
        Slice::FromSnake(snake, properties1, std::nullopt, found_out_of_spec_joint_width, false, false, std::nullopt);
    CHECK(!result);

    auto updated_properties                        = properties1;
    updated_properties.upper_joint_width_tolerance = 2.;

    // Use update joint properties
    result = Slice::FromSnake(snake, updated_properties, std::nullopt, found_out_of_spec_joint_width, true, false,
                              std::nullopt);
    CHECK(result);

    // Check that the points are positioned reasonable
    auto [profile, num_walls, approximation_used] = result.value();
    CHECK(!approximation_used);
    CHECK(num_walls == 0);
    CHECK(profile[0].x - 0.0454 < 0.0001);
    CHECK(profile[0].y - -0.292 < 0.);
    CHECK(profile[6].x - 0.0764 < 0.0001);
    CHECK(profile[6].y - -0.292 < 0.);
    CHECK(profile[0].x < profile[1].x);
    CHECK(profile[1].x < profile[2].x);
    CHECK(profile[2].x < profile[3].x);
    CHECK(profile[3].x < profile[4].x);
    CHECK(profile[4].x < profile[5].x);
    CHECK(profile[5].x < profile[6].x);

    // Use approximation
    std::tuple<double, double> approx = {0.03, 0.06};
    result =
        Slice::FromSnake(snake, updated_properties, std::nullopt, found_out_of_spec_joint_width, true, false, approx);
    CHECK(result);

    // Check that the points are positioned reasonable
    auto [p, nw, a] = result.value();
    CHECK(a);
    CHECK(nw == 0);
    CHECK(p[0].x == 0.03);
    CHECK(p[6].x == 0.06);
    CHECK(profile[0].x < profile[1].x);
    CHECK(profile[1].x < profile[2].x);
    CHECK(profile[2].x < profile[3].x);
    CHECK(profile[3].x < profile[4].x);
    CHECK(profile[4].x < profile[5].x);
    CHECK(profile[5].x < profile[6].x);
  }
  TEST_CASE("FromSnake - real snake cap ") {
    // Better way to initialize a snake from vector?
    WorkspaceCoordinates snake(3, snake_no_walls.size() / 3);
    int index = 0;
    for (int i = 0; i < snake_cap.size() / 3; i++) {
      snake.row(0)[i]  = snake_cap[index];
      snake.row(1)[i]  = snake_cap[index + 1];
      snake.row(2)[i]  = snake_cap[index + 2];
      index           += 3;
    }
    bool found_out_of_spec_joint_width = false;

    auto result =
        Slice::FromSnake(snake, properties1, std::nullopt, found_out_of_spec_joint_width, false, false, std::nullopt);
    CHECK(!result);

    auto approx_abw0_x = 0.06101;
    auto approx_abw6_x = 0.08896;

    // ABW0_x and ABW6_x should be close to these positions
    auto approximation = std::make_tuple(approx_abw0_x, approx_abw6_x);
    // Rerun and use approximation
    result =
        Slice::FromSnake(snake, properties1, std::nullopt, found_out_of_spec_joint_width, false, true, approximation);
    CHECK(result);

    auto [profile, num_walls, approximation_used] = result.value();
    CHECK(approximation_used);
    CHECK(num_walls == 0);
    CHECK(profile[0].x - approx_abw0_x < 0.0001);
    CHECK(profile[0].y - -0.280 < 0.);
    CHECK(profile[6].x - approx_abw6_x < 0.0001);
    CHECK(profile[6].y - -0.280 < 0.);

    CHECK(profile[0].x == profile[1].x);
    CHECK(profile[1].x < profile[2].x);
    CHECK(profile[2].x < profile[3].x);
    CHECK(profile[3].x < profile[4].x);
    CHECK(profile[4].x < profile[5].x);
    CHECK(profile[5].x == profile[6].x);
  }
}
#endif
