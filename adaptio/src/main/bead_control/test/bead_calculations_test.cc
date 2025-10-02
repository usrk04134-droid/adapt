#include "../src/bead_calculations.h"

#include <doctest/doctest.h>

#include <cmath>
#include <Eigen/Eigen>
#include <numbers>

#include "../src/weld_position_data_storage.h"
#include "common/groove/point.h"

// NOLINTBEGIN(*-magic-numbers, misc-include-cleaner)

namespace {
enum SliceSize {
  DECREASING,
  INCREASING,
  EQUAL,
};

}  // namespace

namespace bead_control {
TEST_SUITE("BeadCalculations") {
  TEST_CASE("MeanLayerAreaLeft") {
    common::groove::Point p0 = {.horizontal = 1.5, .vertical = 0.5};
    common::groove::Point p1 = {.horizontal = 0.5, .vertical = -0.5};
    common::groove::Point p2 = {.horizontal = 0.5, .vertical = -0.5};
    common::groove::Point p3 = {.horizontal = 0.5, .vertical = -0.5};
    common::groove::Point p4 = {.horizontal = 0.5, .vertical = -0.5};
    common::groove::Point p5 = {.horizontal = -0.5, .vertical = -0.5};
    common::groove::Point p6 = {.horizontal = -1.5, .vertical = 0.5};

    auto const empty_groove = common::groove::Groove(p0, p1, p2, p3, p4, p5, p6);

    // We would like an bead height of 0.5
    auto bead_area = pow(0.5 / 0.8, 2.) / 2. * std::numbers::pi;

    auto mean_layer_area = BeadCalc::MeanLayerArea(empty_groove, bead_area, bead_area, 0.5);
    CHECK_EQ(mean_layer_area, 0.75);
  }

  TEST_CASE("MeanGrooveWidth") {
    common::groove::Point p0 = {.horizontal = 1.5, .vertical = 0.5};
    common::groove::Point p1 = {.horizontal = 0.5, .vertical = -0.5};
    common::groove::Point p2 = {.horizontal = 0.5, .vertical = -0.5};
    common::groove::Point p3 = {.horizontal = 0.0, .vertical = -0.5};
    common::groove::Point p4 = {.horizontal = -0.5, .vertical = -0.5};
    common::groove::Point p5 = {.horizontal = -0.5, .vertical = -0.5};
    common::groove::Point p6 = {.horizontal = -1.5, .vertical = 0.5};

    auto const empty_groove = common::groove::Groove(p0, p1, p2, p3, p4, p5, p6);
    auto const empty_groove1 =
        common::groove::Groove({.horizontal = 1.0, .vertical = 0.5}, {.horizontal = 0.25, .vertical = -0.5}, p2, p3, p4,
                     {.horizontal = -0.25, .vertical = -0.5}, {.horizontal = -1.0, .vertical = 0.5});

    boost::circular_buffer<WeldPositionDataStorage::Entry> cur_bead(2);

    cur_bead.push_back(WeldPositionDataStorage::Entry{
        .position = 0.0, .data = {.weld_object_lin_velocity = 1.0, .groove = empty_groove, .weld_system1 = {}}
    });
    cur_bead.push_back(WeldPositionDataStorage::Entry{
        .position = 0.0, .data = {.weld_object_lin_velocity = 1.0, .groove = empty_groove1, .weld_system1 = {}}
    });

    WeldPositionDataStorage::Slice cur_sub_container(cur_bead.begin(), 2);

    auto mean_width = BeadCalc::MeanLowerGrooveWidth(cur_sub_container);
    CHECK_EQ(mean_width, 0.75);
  }

  TEST_CASE("BeadArea") {
    auto bead_area  = BeadCalc::BeadArea(1.0, 2.0, 5.0);
    bead_area      += BeadCalc::BeadArea(3.0, 4.0, 5.0);
    CHECK(bead_area == doctest::Approx((std::numbers::pi *
                                        (1.0 * std::pow((2.0 / 2.), 2.) + 3.0 * std::pow((4.0 / 2.), 2.)) / 5.0)));
  }

  TEST_CASE("MeanBeadArea") {
    // velocity rad/second. one turn in one hour
    double wire_diameter            = 3.;
    double wire_lin_velocity        = 2.;
    double weld_object_radius       = 7000.;
    double weld_object_ang_velocity = 2. * std::numbers::pi * weld_object_radius / 3600.;

    boost::circular_buffer<WeldPositionDataStorage::Entry> cur_bead(20. * 3600.);

    // Number of stored element for one turn is 20 per second * 3600
    for (int i = 0; i < 20 * 3600; i++) {
      cur_bead.push_back(WeldPositionDataStorage::Entry{
          .position = 0.0,
          .data     = {.weld_object_lin_velocity = weld_object_ang_velocity,
                       .groove                   = {},
                       .weld_system1             = {.current = 1.0, .wire_lin_velocity = wire_lin_velocity},
                       .weld_system2             = {.current = 2.0, .wire_lin_velocity = wire_lin_velocity}}
      });
    }

    WeldPositionDataStorage::Slice cur_sub_container(cur_bead.begin(), 20 * 3600);

    auto bead_area = BeadCalc::MeanBeadArea(cur_sub_container, wire_diameter, false, wire_diameter, false);

    CHECK(bead_area ==
          doctest::Approx((std::numbers::pi * (2. * wire_lin_velocity * std::pow((wire_diameter / 2.), 2.)) /
                           weld_object_ang_velocity)));

    bead_area = BeadCalc::MeanBeadArea(cur_sub_container, wire_diameter, true, wire_diameter, false);

    CHECK(bead_area ==
          doctest::Approx((std::numbers::pi * (3. * wire_lin_velocity * std::pow((wire_diameter / 2.), 2.)) /
                           weld_object_ang_velocity)));

    bead_area = BeadCalc::MeanBeadArea(cur_sub_container, wire_diameter, true, wire_diameter, true);

    CHECK(bead_area ==
          doctest::Approx((std::numbers::pi * (4. * wire_lin_velocity * std::pow((wire_diameter / 2.), 2.)) /
                           weld_object_ang_velocity)));
  }

  TEST_CASE("DistanceToAngle") {
    auto radius = 1.0;
    CHECK(BeadCalc::Distance2Angle(radius, 2 * std::numbers::pi * radius) == doctest::Approx(2 * std::numbers::pi));
  }

  TEST_CASE("BeadSliceAreaRatio") {
    auto const tests = std::vector<std::pair<SliceSize, common::groove::Groove>>{
        // Top line horizontal
        // Bottom line horizontal
        {
         SliceSize::EQUAL,
         {
                {.horizontal = 75., .vertical = 25.},
                {.horizontal = 25., .vertical = -25.},
                {.horizontal = 12.5, .vertical = -25.},
                {.horizontal = 0., .vertical = -25.},
                {.horizontal = -12.5, .vertical = -25.},
                {.horizontal = -25, .vertical = -25.},
                {.horizontal = -75, .vertical = 25.},
            }, },

        // Top line upward slope
        // Bottom line horizontal
        {
         SliceSize::INCREASING,
         {
                {.horizontal = 75., .vertical = 25.},
                {.horizontal = 25., .vertical = -25.},
                {.horizontal = 12.5, .vertical = -25.},
                {.horizontal = 0., .vertical = -25.},
                {.horizontal = -12.5, .vertical = -25.},
                {.horizontal = -25, .vertical = -25.},
                {.horizontal = -75, .vertical = 50.},
            }, },

        // Top line downward slope
        // Bottom line horizontal
        {
         SliceSize::DECREASING,
         {
                {.horizontal = 75., .vertical = 25.},
                {.horizontal = 25., .vertical = -25.},
                {.horizontal = 12.5, .vertical = -25.},
                {.horizontal = 0., .vertical = -25.},
                {.horizontal = -12.5, .vertical = -25.},
                {.horizontal = -25, .vertical = -25.},
                {.horizontal = -75, .vertical = 10.},
            }, },

        // Top line horizontal
        // Bottom line downward slope
        {
         SliceSize::INCREASING,
         {
                {.horizontal = 75., .vertical = 25.},
                {.horizontal = 25., .vertical = -26.},
                {.horizontal = 12.5, .vertical = -27.},
                {.horizontal = 0., .vertical = -28.},
                {.horizontal = -12.5, .vertical = -29.},
                {.horizontal = -25, .vertical = -29.},
                {.horizontal = -75, .vertical = 25.},
            }, },

        // Top line horizontal
        // Bottom line upward slope
        {
         SliceSize::DECREASING,
         {
                {.horizontal = 75., .vertical = 25.},
                {.horizontal = 25., .vertical = -25.},
                {.horizontal = 12.5, .vertical = -20.},
                {.horizontal = 0., .vertical = -15.},
                {.horizontal = -12.5, .vertical = -10.},
                {.horizontal = -25, .vertical = -5.},
                {.horizontal = -75, .vertical = 25.},
            }, },

        // Top line upward slope
        // Bottom line upward slope (parallel with topline)
        {
         SliceSize::EQUAL,
         {
                {.horizontal = 75., .vertical = 0.},
                {.horizontal = 25., .vertical = -10.},
                {.horizontal = 12.5, .vertical = -7.5},
                {.horizontal = 0., .vertical = -5.},
                {.horizontal = -12.5, .vertical = -2.5},
                {.horizontal = -25, .vertical = 0.},
                {.horizontal = -75, .vertical = 30.},
            }, },

        // Top downward slope
        // Bottom line upward slope
        {
         SliceSize::DECREASING,
         {
                {.horizontal = 75., .vertical = 25.},
                {.horizontal = 25., .vertical = -25.},
                {.horizontal = 12.5, .vertical = -20.},
                {.horizontal = 0., .vertical = -15.},
                {.horizontal = -12.5, .vertical = -10.},
                {.horizontal = -25, .vertical = -5.},
                {.horizontal = -75, .vertical = 15.},
            }, },

        // Top upward slope
        // Bottom line downward slope
        {
         SliceSize::INCREASING,
         {
                {.horizontal = 75., .vertical = 15.},
                {.horizontal = 25., .vertical = -10.},
                {.horizontal = 12.5, .vertical = -11.},
                {.horizontal = 0., .vertical = -15.},
                {.horizontal = -12.5, .vertical = -20.},
                {.horizontal = -25, .vertical = -22.},
                {.horizontal = -75, .vertical = 25.},
            }, },
    };

    for (const auto& test : tests) {
      for (int beads = 2; beads <= 10; ++beads) {
        // calculate bead slice position

        auto total = 0.;
        auto last  = 0.;
        for (auto bead = 1; bead <= beads; ++bead) {
          auto const res = BeadCalc::BeadSliceAreaRatio(test.second, bead, beads);

          if (test.first == SliceSize::EQUAL) {
            REQUIRE_EQ(res, doctest::Approx(1.));
          } else if (test.first == SliceSize::DECREASING && bead > 1) {
            CHECK(res < last);
          } else if (test.first == SliceSize::INCREASING && bead > 1) {
            CHECK(res > last);
          }

          total += res;
          last   = res;
        }
        REQUIRE_EQ(total, doctest::Approx(beads));
      }
    }
  }

  TEST_CASE("BeadPositionAdjustment") {
    auto const tests = std::vector<std::pair<SliceSize, common::groove::Groove>>{
        // Top line horizontal
        // Bottom line horizontal
        {
         SliceSize::EQUAL,
         {
                {.horizontal = 75., .vertical = 25.},
                {.horizontal = 25., .vertical = -25.},
                {.horizontal = 12.5, .vertical = -25.},
                {.horizontal = 0., .vertical = -25.},
                {.horizontal = -12.5, .vertical = -25.},
                {.horizontal = -25, .vertical = -25.},
                {.horizontal = -75, .vertical = 25.},
            }, },

        // Top line upward slope
        // Bottom line horizontal
        {
         SliceSize::INCREASING,
         {
                {.horizontal = 75., .vertical = 25.},
                {.horizontal = 25., .vertical = -25.},
                {.horizontal = 12.5, .vertical = -25.},
                {.horizontal = 0., .vertical = -25.},
                {.horizontal = -12.5, .vertical = -25.},
                {.horizontal = -25, .vertical = -25.},
                {.horizontal = -75, .vertical = 50.},
            }, },

        // Top line downward slope
        // Bottom line horizontal
        {
         SliceSize::DECREASING,
         {
                {.horizontal = 75., .vertical = 25.},
                {.horizontal = 25., .vertical = -25.},
                {.horizontal = 12.5, .vertical = -25.},
                {.horizontal = 0., .vertical = -25.},
                {.horizontal = -12.5, .vertical = -25.},
                {.horizontal = -25, .vertical = -25.},
                {.horizontal = -75, .vertical = 10.},
            }, },

        // Top line horizontal
        // Bottom line downward slope
        {
         SliceSize::INCREASING,
         {
                {.horizontal = 75., .vertical = 25.},
                {.horizontal = 25., .vertical = -21.},
                {.horizontal = 12.5, .vertical = -22.},
                {.horizontal = 0., .vertical = -23.},
                {.horizontal = -12.5, .vertical = -24.},
                {.horizontal = -25, .vertical = -25.},
                {.horizontal = -75, .vertical = 25.},
            }, },

        // Top line horizontal
        // Bottom line upward slope
        {
         SliceSize::DECREASING,
         {
                {.horizontal = 75., .vertical = 25.},
                {.horizontal = 25., .vertical = -25.},
                {.horizontal = 12.5, .vertical = -24.},
                {.horizontal = 0., .vertical = -23.},
                {.horizontal = -12.5, .vertical = -22.},
                {.horizontal = -25, .vertical = -21.},
                {.horizontal = -75, .vertical = 25.},
            }, },

        // Top line upward slope
        // Bottom line upward slope (parallel with topline)
        {
         SliceSize::EQUAL,
         {
                {.horizontal = 75., .vertical = 25.},
                {.horizontal = 25., .vertical = -10.},
                {.horizontal = 12.5, .vertical = -9.},
                {.horizontal = 0., .vertical = -8.},
                {.horizontal = -12.5, .vertical = -7.},
                {.horizontal = -25, .vertical = -6.},
                {.horizontal = -75, .vertical = 37.},
            }, },

        // Top downward slope
        // Bottom line upward slope
        {
         SliceSize::DECREASING,
         {
                {.horizontal = 75., .vertical = 25.},
                {.horizontal = 25., .vertical = -25.},
                {.horizontal = 12.5, .vertical = -22.},
                {.horizontal = 0., .vertical = -20.},
                {.horizontal = -12.5, .vertical = -18.},
                {.horizontal = -25, .vertical = -16.},
                {.horizontal = -75, .vertical = 20.},
            }, },

        // Top upward slope
        // Bottom line downward slope
        {
         SliceSize::INCREASING,
         {
                {.horizontal = 75., .vertical = 15.},
                {.horizontal = 25., .vertical = -10.},
                {.horizontal = 12.5, .vertical = -11.},
                {.horizontal = 0., .vertical = -15.},
                {.horizontal = -12.5, .vertical = -20.},
                {.horizontal = -25, .vertical = -22.},
                {.horizontal = -75, .vertical = 25.},
            }, },
    };

    for (const auto& test : tests) {
      auto const new_pos = BeadCalc::BeadPositionAdjustment(test.second, 0.5, 2.);

      switch (test.first) {
        case SliceSize::DECREASING:
          CHECK(new_pos < 0.5);
          break;
        case SliceSize::INCREASING:
          CHECK(new_pos > 0.5);
          break;
        case SliceSize::EQUAL:
          CHECK(new_pos == doctest::Approx(0.5));
          break;
        default:
          abort();
      }
    }
  }
}

}  // namespace bead_control
// NOLINTEND(*-magic-numbers, misc-include-cleaner)
