#include "../bead_control.h"

#include <doctest/doctest.h>

#include <cmath>
#include <numbers>

#include "../src/bead_control_impl.h"
#include "bead_control/bead_control_types.h"
#include "macs/macs_groove.h"
#include "macs/macs_point.h"
#include "test_utils/testlog.h"
#include "tracking/tracking_manager.h"

// NOLINTBEGIN(*-magic-numbers, misc-include-cleaner)

TEST_SUITE("BeadControl") {
  TEST_CASE("HorizontalVelocity") {
    auto const weld_speed = 100.; /* cm/min */
    auto const angle_deg  = 30.;
    auto const angle_rad  = angle_deg * static_cast<double>(std::numbers::pi) / 180.;
    auto const radius     = 3500.;

    auto const weld_speed0 = weld_speed * 10. / 60. / radius; /* rad/sec */

    auto const result = bead_control::GetRepositionHorizontalVelocity(weld_speed0, radius, angle_rad); /* mm/sec */

    auto const expect = weld_speed * tan(angle_rad);

    REQUIRE_EQ(result / 10. * 60., doctest::Approx(expect));
  }

  TEST_CASE("HorizontalVelocity2") {
    auto const weld_speed = 80.; /* cm/min */
    auto const angle_deg  = 15.;
    auto const angle_rad  = angle_deg * static_cast<double>(std::numbers::pi) / 180.;
    auto const radius     = 5000.;

    auto const weld_speed0 = weld_speed * 10. / 60. / radius; /* rad/sec */

    auto const result = bead_control::GetRepositionHorizontalVelocity(weld_speed0, radius, angle_rad); /* mm/sec */

    auto const expect = weld_speed * tan(angle_rad);

    REQUIRE_EQ(result / 10. * 60., doctest::Approx(expect));
  }

  TEST_CASE("Update") {
    bead_control::WeldPositionDataStorage storage(bead_control::MAX_BUFFER_SIZE);
    bead_control::BeadControlImpl control(&storage, std::chrono::steady_clock::now);

    // Groove width: 150mm
    // Groove lower width: 50mm
    // Wall angle: 45 degrees
    macs::Point p0 = {.horizontal = 75., .vertical = 25.};
    macs::Point p1 = {.horizontal = 25., .vertical = -25.};
    macs::Point p2 = {.horizontal = 12.5, .vertical = -25.};
    macs::Point p3 = {.horizontal = 0., .vertical = -25.};
    macs::Point p4 = {.horizontal = -12.5, .vertical = -25.};
    macs::Point p5 = {.horizontal = -25, .vertical = -25.};
    macs::Point p6 = {.horizontal = -75, .vertical = 25.};

    auto const groove = macs::Groove(p0, p1, p2, p3, p4, p5, p6);

    auto weld_object_lin_velocity          = 1000. / 60.;  // mm/sec
    auto wire_lin_velocity                 = 6000. / 60.;  // mm/sec
    auto weld_object_radius                = 1500.;
    bead_control::BeadControl::Input input = {
        .weld_object_angle        = 1.2, // Start angle can be any between 0 - 2*pi
        .weld_object_ang_velocity = weld_object_lin_velocity / weld_object_radius,
        .weld_object_radius       = weld_object_radius,
        .weld_system1             = {.wire_lin_velocity = wire_lin_velocity,
                                     .current           = 1.,
                                     .wire_diameter     = 3.,
                                     .twin_wire         = false},
        .weld_system2             = {.wire_lin_velocity = wire_lin_velocity,
                                     .current           = 1.,
                                     .wire_diameter     = 3.,
                                     .twin_wire         = false},
        .groove                   = groove,
        .steady_satisfied         = true,
    };

    // Step in radians between each sample. New ABW points every 50ms i.e. 20 times/s
    auto angle_step = input.weld_object_ang_velocity / 20.;

    // Number of samples for one turn
    auto number_of_samples = static_cast<int>(2 * std::numbers::pi / angle_step);

    auto test_controller_update = [&control, &input, angle_step](int bead, int layer, tracking::TrackingMode tracking,
                                                                 bead_control::State state) {
      auto [result, output] = control.Update(input);

      CHECK(result == bead_control::BeadControl::Result::OK);
      CHECK(output.tracking_mode == tracking);

      input.weld_object_angle = std::fmod(input.weld_object_angle + angle_step, 2 * std::numbers::pi);

      auto status = control.GetStatus();
      CHECK(status.bead_number == bead);
      CHECK(status.layer_number == layer);
      CHECK(status.state == state);
    };

    control.SetWallOffset(3.);

    // ABP started
    test_controller_update(1, 1, tracking::TrackingMode::TRACKING_LEFT_HEIGHT, bead_control::State::REPOSITIONING);
    // Reposition to left side
    // The axis is half way
    input.steady_satisfied = false;
    test_controller_update(1, 1, tracking::TrackingMode::TRACKING_LEFT_HEIGHT, bead_control::State::REPOSITIONING);

    // The axis is within tolerance
    input.steady_satisfied = true;

    CHECK(!control.GetEmptyGroove(std::numbers::pi).has_value());

    // Welding first bead
    for (int i = 0; i <= number_of_samples; i++) {
      test_controller_update(1, 1, tracking::TrackingMode::TRACKING_LEFT_HEIGHT, bead_control::State::STEADY);
    }

    // First bead is ready
    test_controller_update(1, 1, tracking::TrackingMode::TRACKING_LEFT_HEIGHT, bead_control::State::OVERLAPPING);

    // Reposition to right side
    test_controller_update(2, 1, tracking::TrackingMode::TRACKING_RIGHT_HEIGHT, bead_control::State::REPOSITIONING);

    CHECK(control.GetEmptyGroove(std::numbers::pi).has_value());

    // The axis is in the middle of the groove
    input.steady_satisfied = false;
    test_controller_update(2, 1, tracking::TrackingMode::TRACKING_RIGHT_HEIGHT, bead_control::State::REPOSITIONING);

    // Axis is within tolerance
    input.steady_satisfied = true;

    // Welding second bead
    // Scanner starts to see the first bead
    // Set height so that there can be 3 beads in layer
    // Height is calculated from weld parameters
    // bead area = pi * wire velocity * (wire diameter/2)^2 / (1000/60) * 2 = 84.82mm2
    // bead height = 0.8 * sqrt(2*84.82/pi) = 5.87mm
    // layer area = 5.87*50 + 5.87*5.87 = 327.96mm2
    // num of berads = 327.96 / 84.82 = 3.87 -> 3 beads in layer

    for (int i = 0; i <= number_of_samples; i++) {
      test_controller_update(2, 1, tracking::TrackingMode::TRACKING_RIGHT_HEIGHT, bead_control::State::STEADY);
    }

    // Second bead is ready
    test_controller_update(2, 1, tracking::TrackingMode::TRACKING_RIGHT_HEIGHT, bead_control::State::OVERLAPPING);

    test_controller_update(3, 1, tracking::TrackingMode::TRACKING_CENTER_HEIGHT, bead_control::State::REPOSITIONING);

    for (int i = 0; i <= number_of_samples; i++) {
      test_controller_update(3, 1, tracking::TrackingMode::TRACKING_CENTER_HEIGHT, bead_control::State::STEADY);
    }
    // Third bead and layer is ready
    test_controller_update(3, 1, tracking::TrackingMode::TRACKING_CENTER_HEIGHT, bead_control::State::OVERLAPPING);

    test_controller_update(1, 2, tracking::TrackingMode::TRACKING_LEFT_HEIGHT, bead_control::State::REPOSITIONING);
  }

  TEST_CASE("BeadAdaptivity") {
    struct TestParams {
      double k_gain;
      double bead3_pos;
      double wall_offset;
      macs::Groove groove;
      std::vector<double> bead_slice_area_ratio;
    };

    auto const tests = {
        /* uneven top surface - high right side
         * - k_gain=0 -> no bead placement adjustment
         * - bead_slice_area_ratio increasing */
        TestParams{.k_gain                = 0.,
                   .bead3_pos             = .50,
                   .wall_offset           = 3.,
                   .groove                = {{.horizontal = 75., .vertical = 25.},
                                             {.horizontal = 25., .vertical = -25.},
                                             {.horizontal = 12.5, .vertical = -25.},
                                             {.horizontal = 0., .vertical = -25.},
                                             {.horizontal = -12.5, .vertical = -25.},
                                             {.horizontal = -25, .vertical = -25.},
                                             {.horizontal = -75, .vertical = 45.}},
                   .bead_slice_area_ratio = {1.0, 1.042, 1.}},

        /* uneven top surface - high right side
         * - k_gain=2 -> middle bead adjusted to right side of groove
         * - bead_slice_area_ratio increasing */
        TestParams{.k_gain                = 2.,
                   .bead3_pos             = .5844,
                   .wall_offset           = 4.,
                   .groove                = {{.horizontal = 75., .vertical = 25.},
                                             {.horizontal = 25., .vertical = -25.},
                                             {.horizontal = 12.5, .vertical = -25.},
                                             {.horizontal = 0., .vertical = -25.},
                                             {.horizontal = -12.5, .vertical = -25.},
                                             {.horizontal = -25, .vertical = -25.},
                                             {.horizontal = -75, .vertical = 45.}},
                   .bead_slice_area_ratio = {1.0, 1.042, 1.}},

        /* uneven top surface - high right side
         * - k_gain=3 -> middle bead adjusted to right side of groove
         * - bead_slice_area_ratio increasing */
        TestParams{.k_gain                = 3.,
                   .bead3_pos             = .6211,
                   .wall_offset           = 4.,
                   .groove                = {{.horizontal = 75., .vertical = 25.},
                                             {.horizontal = 25., .vertical = -25.},
                                             {.horizontal = 12.5, .vertical = -25.},
                                             {.horizontal = 0., .vertical = -25.},
                                             {.horizontal = -12.5, .vertical = -25.},
                                             {.horizontal = -25, .vertical = -25.},
                                             {.horizontal = -75, .vertical = 45.}},
                   .bead_slice_area_ratio = {1., 1.042, 1.} },

        /* uneven top surface - high left side
         * - k_gain=2 -> middle bead adjusted to left side of groove
         * - bead_slice_area_ratio decreasing */
        TestParams{.k_gain                = 3.,
                   .bead3_pos             = .4353,
                   .wall_offset           = 2.,
                   .groove                = {{.horizontal = 75., .vertical = 35.},
                                             {.horizontal = 15., .vertical = -25.},
                                             {.horizontal = 5, .vertical = -25.},
                                             {.horizontal = -5., .vertical = -25.},
                                             {.horizontal = -15, .vertical = -25.},
                                             {.horizontal = -25, .vertical = -25.},
                                             {.horizontal = -75, .vertical = 25.}},
                   .bead_slice_area_ratio = {1.0, 0.981, 1.}},
    };

    for (auto test : tests) {
      bead_control::WeldPositionDataStorage storage(bead_control::MAX_BUFFER_SIZE);
      bead_control::BeadControlImpl control(&storage, std::chrono::steady_clock::now);

      control.SetKGain(test.k_gain);
      control.SetWallOffset(test.wall_offset);

      auto weld_object_lin_velocity          = 1000. / 60.;  // mm/sec
      auto wire_lin_velocity                 = 5000. / 60.;  // mm/sec
      auto weld_object_radius                = 1500.;
      bead_control::BeadControl::Input input = {
          .weld_object_angle        = 1.2, // Start angle can be any between 0 - 2*pi
          .weld_object_ang_velocity = weld_object_lin_velocity / weld_object_radius,
          .weld_object_radius       = weld_object_radius,
          .weld_system1             = {.wire_lin_velocity = wire_lin_velocity,
                                       .current           = 1.,
                                       .wire_diameter     = 3.,
                                       .twin_wire         = false},
          .weld_system2             = {.wire_lin_velocity = wire_lin_velocity,
                                       .current           = 1.,
                                       .wire_diameter     = 3.,
                                       .twin_wire         = false},
          .groove                   = test.groove,
          .steady_satisfied         = true,
      };

      // Step in radians between each sample. New ABW points every 50ms i.e. 20 times/s
      // auto angle_step = input.weld_object_ang_velocity / 20;
      auto angle_step = 2 * std::numbers::pi / 6.;

      // Number of samples for one turn
      auto number_of_samples = static_cast<int>(2 * std::numbers::pi / angle_step);

      auto test_controller_update = [&control, &input, angle_step](
                                        int bead, int layer, tracking::TrackingMode tracking, bead_control::State state,
                                        std::optional<double> bead_pos, double bead_slice_area_ratio) {
        auto [result, output] = control.Update(input);
        auto status           = control.GetStatus();

        CHECK(result == bead_control::BeadControl::Result::OK);
        CHECK(output.tracking_mode == tracking);
        if (bead_pos) {
          REQUIRE(output.horizontal_offset == doctest::Approx(bead_pos.value()).epsilon(0.05));
        }
        REQUIRE(output.bead_slice_area_ratio == doctest::Approx(bead_slice_area_ratio).epsilon(0.001));

        input.weld_object_angle = std::fmod(input.weld_object_angle + angle_step, 2 * std::numbers::pi);

        CHECK(status.bead_number == bead);
        CHECK(status.layer_number == layer);
        CHECK(status.state == state);
      };

      // ABP started
      test_controller_update(1, 1, tracking::TrackingMode::TRACKING_LEFT_HEIGHT, bead_control::State::REPOSITIONING,
                             test.wall_offset, 1.);
      // Reposition to left side
      // The axis is half way
      input.steady_satisfied = false;
      test_controller_update(1, 1, tracking::TrackingMode::TRACKING_LEFT_HEIGHT, bead_control::State::REPOSITIONING,
                             test.wall_offset, 1.);

      // The axis is within tolerance
      input.steady_satisfied = true;

      // Welding first bead
      for (int i = 0; i <= number_of_samples; i++) {
        test_controller_update(1, 1, tracking::TrackingMode::TRACKING_LEFT_HEIGHT, bead_control::State::STEADY,
                               test.wall_offset, test.bead_slice_area_ratio[0]);
      }

      // First bead is ready
      test_controller_update(1, 1, tracking::TrackingMode::TRACKING_LEFT_HEIGHT, bead_control::State::OVERLAPPING, {},
                             1.);

      // Reposition to right side
      test_controller_update(2, 1, tracking::TrackingMode::TRACKING_RIGHT_HEIGHT, bead_control::State::REPOSITIONING,
                             test.wall_offset, 1.);

      // The axis is in the middle of the groove
      input.steady_satisfied = false;
      test_controller_update(2, 1, tracking::TrackingMode::TRACKING_RIGHT_HEIGHT, bead_control::State::REPOSITIONING,
                             test.wall_offset, 1.);

      // Axis is within tolerance
      input.steady_satisfied = true;

      // Welding second bead
      // Scanner starts to see the first bead
      // Set height so that there can be 3 beads in layer
      // Height is calculated from weld parameters

      for (int i = 0; i <= number_of_samples; i++) {
        test_controller_update(2, 1, tracking::TrackingMode::TRACKING_RIGHT_HEIGHT, bead_control::State::STEADY,
                               test.wall_offset, test.bead_slice_area_ratio[1]);
      }

      // Second bead is ready
      test_controller_update(2, 1, tracking::TrackingMode::TRACKING_RIGHT_HEIGHT, bead_control::State::OVERLAPPING, {},
                             1.);

      auto const bot_groove_width =
          (test.groove[macs::ABW_LOWER_LEFT].horizontal - test.groove[macs::ABW_LOWER_RIGHT].horizontal) -
          (2 * test.wall_offset);
      auto const horizontal_offset = (bot_groove_width * test.bead3_pos) - (bot_groove_width / 2.);
      test_controller_update(3, 1, tracking::TrackingMode::TRACKING_CENTER_HEIGHT, bead_control::State::REPOSITIONING,
                             {}, 1.);
      return;

      for (int i = 0; i <= number_of_samples; i++) {
        test_controller_update(3, 1, tracking::TrackingMode::TRACKING_CENTER_HEIGHT, bead_control::State::STEADY,
                               horizontal_offset, test.bead_slice_area_ratio[2]);
      }
      // Third bead and layer is ready
      test_controller_update(3, 1, tracking::TrackingMode::TRACKING_CENTER_HEIGHT, bead_control::State::OVERLAPPING, {},
                             1.);

      test_controller_update(1, 2, tracking::TrackingMode::TRACKING_LEFT_HEIGHT, bead_control::State::REPOSITIONING, {},
                             1.);
    }
  }

  TEST_CASE("CAP bead placement") {
    struct TestParams {
      std::string info;
      macs::Groove groove;
      double cap_corner_offset;
      int cap_beads;
      std::vector<double> bead_positions;
    };

    auto const tests = {
        TestParams{.info              = "CAP with uneven number of beads and positive offset",
                   .groove            = {{.horizontal = 75., .vertical = 25.},
                                         {.horizontal = 70., .vertical = 23.},
                                         {.horizontal = 12.5, .vertical = 23.},
                                         {.horizontal = 0., .vertical = 23.},
                                         {.horizontal = -12.5, .vertical = 23.},
                                         {.horizontal = -70, .vertical = 23.},
                                         {.horizontal = -75, .vertical = 25.}},
                   .cap_corner_offset = 5,
                   .cap_beads         = 5,
                   .bead_positions    = {-70.0, -35.0, 0.0, 35.0, 70.0}             },
        TestParams{.info              = "CAP with even number of beads and negative offset",
                   .groove            = {{.horizontal = 75., .vertical = 25.},
                                         {.horizontal = 70., .vertical = 23.},
                                         {.horizontal = 12.5, .vertical = 23.},
                                         {.horizontal = 0., .vertical = 23.},
                                         {.horizontal = -12.5, .vertical = 23.},
                                         {.horizontal = -70, .vertical = 23.},
                                         {.horizontal = -75, .vertical = 25.}},
                   .cap_corner_offset = -5,
                   .cap_beads         = 4,
                   .bead_positions    = {-80.0, -26.667, 26.667, 80.0}              },
        TestParams{.info              = "CAP with uneven number of beads and 0 offset",
                   .groove            = {{.horizontal = 75., .vertical = 25.},
                                         {.horizontal = 70., .vertical = 23.},
                                         {.horizontal = 12.5, .vertical = 23.},
                                         {.horizontal = 0., .vertical = 23.},
                                         {.horizontal = -12.5, .vertical = 23.},
                                         {.horizontal = -70, .vertical = 23.},
                                         {.horizontal = -75, .vertical = 25.}},
                   .cap_corner_offset = 0,
                   .cap_beads         = 7,
                   .bead_positions    = {-75.0, -50.0, -25.0, 0.0, 25.0, 50.0, 75.0}},
    };

    for (auto test : tests) {
      TESTLOG("testcase: {}", test.info);
      bead_control::WeldPositionDataStorage storage(bead_control::MAX_BUFFER_SIZE);
      bead_control::BeadControlImpl control(&storage, std::chrono::steady_clock::now);

      control.SetCapBeads(test.cap_beads);
      control.SetCapCornerOffset(test.cap_corner_offset);
      control.NextLayerCap();

      auto weld_object_lin_velocity          = 1000. / 60.;  // mm/sec
      auto weld_object_radius                = 1500.;
      bead_control::BeadControl::Input input = {
          .weld_object_angle        = 1.2,  // Start angle can be any between 0 - 2*pi
          .weld_object_ang_velocity = weld_object_lin_velocity / weld_object_radius,
          .weld_object_radius       = weld_object_radius,
          .groove                   = test.groove,
          .steady_satisfied         = true,
      };

      // Step in radians between each sample. New ABW points every 50ms i.e. 20 times/s
      auto angle_step = 2 * std::numbers::pi / 6.;

      // Number of samples for one turn
      auto number_of_samples = static_cast<int>(2 * std::numbers::pi / angle_step);

      auto test_controller_update = [&control, &input, angle_step](int bead, bead_control::State state,
                                                                   std::optional<double> bead_pos) {
        auto [result, output] = control.Update(input);
        auto status           = control.GetStatus();

        CHECK(result == bead_control::BeadControl::Result::OK);
        CHECK(output.tracking_mode == tracking::TrackingMode::TRACKING_CENTER_HEIGHT);
        if (bead_pos) {
          REQUIRE(output.horizontal_offset == doctest::Approx(bead_pos.value()).epsilon(0.05));
        }
        REQUIRE(output.bead_slice_area_ratio == doctest::Approx(1.0).epsilon(0.001));
        REQUIRE(output.groove_area_ratio == doctest::Approx(1.0).epsilon(0.001));

        input.weld_object_angle = std::fmod(input.weld_object_angle + angle_step, 2 * std::numbers::pi);

        CHECK(status.bead_number == bead);
        CHECK(status.layer_number == 1);
        CHECK(status.state == state);
      };

      for (auto bead = 1; bead <= test.cap_beads; ++bead) {
        auto const pos = test.bead_positions[bead - 1];
        test_controller_update(bead, bead_control::State::REPOSITIONING, pos);
        input.steady_satisfied = false;
        test_controller_update(bead, bead_control::State::REPOSITIONING, pos);

        // The axis is within tolerance
        input.steady_satisfied = true;

        for (int i = 0; i <= number_of_samples; i++) {
          test_controller_update(bead, bead_control::State::STEADY, pos);
        }

        test_controller_update(bead, bead_control::State::OVERLAPPING, pos);
      }

      /* send one additional input to trigger new bead and FINISHED groove */
      auto [result, output] = control.Update(input);
      CHECK(result == bead_control::BeadControl::Result::FINISHED);
    }
  }
}
// NOLINTEND(*-magic-numbers, misc-include-cleaner)
